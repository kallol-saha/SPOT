import logging
import os
from pathlib import Path
from typing import (
    Any,
    Callable,
    Dict,
    Generic,
    List,
    Optional,
    Protocol,
    Sequence,
    Tuple,
    Type,
    TypeVar,
    Union,
    runtime_checkable,
)

import numpy as np
import torch
import torch.utils.data as td
import torch_geometric.data as tgd
from rpad.core.distributed import NPSeed, distributed_eval


@runtime_checkable
class CanSample(Protocol):
    """This is just a dataset which has the 'get_data' function. This function doesn't
    need to be deterministic, for instance if we want to randomly sample viewpoints."""

    def get_data(self, *args, seed: NPSeed = None) -> tgd.Data:
        """The"""


class SinglePathDataset(tgd.InMemoryDataset):
    """Dataset consisting of a single file, generated in a different process."""

    def __init__(
        self, processed_path: str, transform=None, pre_transform=None, pre_filter=None
    ):
        root = Path(processed_path).parent
        super().__init__(root, transform, pre_transform, pre_filter, log=False)
        self.data, self.slices = torch.load(processed_path)

    @staticmethod
    def write(data_list: Sequence[tgd.Data], data_path: str):
        """Write a list of data elements into a single file on disk.

        Args:
            data_list (Sequence[tgd.Data]): A list of Data objects.
            data_path (str): The file to write to.
        """
        data, slices = tgd.InMemoryDataset.collate(data_list)
        torch.save((data, slices), data_path)


__dataset: Optional[CanSample] = None


def _init_dataset(dset_cls, dset_kwargs):
    global __dataset
    __dataset = dset_cls(**dset_kwargs)
    assert isinstance(dset_cls, CanSample)


def _parallel_sample(
    get_data_args: Tuple,
    n_repeat: int,
    processed_path: str,
    seed: NPSeed,
):
    assert len(get_data_args) >= 1

    global __dataset
    assert __dataset is not None

    # The output should go in a file that indexes exactly what params
    # were used to generate it.
    prefix = "_".join(get_data_args)
    data_file = f"{prefix}_{n_repeat}.pt"
    data_path = os.path.join(processed_path, data_file)

    if os.path.exists(data_path):
        logging.info(f"{data_path} already exists, skipping...")
        return

    try:
        # Sample the data.
        data_list = []
        # Give each sample a different seed.
        seeds = np.random.default_rng(seed).integers(0, 2**32, size=n_repeat)
        for i in range(n_repeat):
            data_list.append(__dataset.get_data(*get_data_args, seed=seeds[i]))

        # Save it in an in-memory dataset.
        SinglePathDataset.write(data_list, data_path)

    except Exception as err:
        logging.error(f"unable to sample get_data({get_data_args}): {err}")
        raise ValueError(f"unable to sample get_data({get_data_args}): {err}")


def parallel_sample(
    dset_cls: Type[CanSample],
    dset_kwargs: Dict[str, Any],
    get_data_args: List[Tuple],
    processed_path: str,
    n_repeat: int,
    n_proc: int = -1,
    n_workers: int = -1,
    n_proc_per_worker: int = 2,
    seed: Optional[int] = None,
):
    """Run parallel sampling in a dataset. Assumes that we're operating on a
    partnet-style dataset, where we have a unique object which may be sampled
    many different times (i.e. different point clouds, different camera positions)

    This works across dataset types; the only restriction on the dataset is that
    it implements CanSample.

    Each object will be sampled n_repeat times, and collated into a single file
    for each object, placed in the processed_dir (defined by the dataset). This
    filename is <OBJ_ID>_<N_REPEAT>.pt

    For downstream datasets, you can load each file into a SingleObjDataset,
    and then use torch.utils.data.ConcatDataset to concatentate them.

    Args:
        dset_cls (Type[CanSample]): The constructor for the dataset.
        dset_kwargs (Dict): The kwargs to pass to the dataset constructor.
        get_data_args (List[Tuple]): The arguments to pass to each call to get_data. If there
            are N distinct objects in the dataset, this parameter should have the form:

            [(OBJ_ID, arg_1, ...), ...]

            where the length of the list is N, and the first entry in each tuple is the
            object ID (a string). These arguments will be passed to get_data via *get_data_args.
        n_repeat (int, optional): Number of times to call get_data on the same args
            (i.e. when get_data is nondeterministic, to sample viewpoints, etc.). Defaults to 100.
        n_proc (int, optional): Number of processes for multiprocessing.
            If 0, multiprocessing won't be used at all (good for debugging serially).
            If -1, uses as many CPUs as are available.
            Defaults to -1.

    Raises:
        ValueError: If any sampling fails, this method will raise an exception.
    """
    # task_number, args, n_repeat, processed_dir, seed
    ps_args = [
        {
            "get_data_args": args,
            "n_repeat": n_repeat,
            "processed_path": processed_path,
        }
        for args in get_data_args
    ]

    results, completeds = distributed_eval(
        _parallel_sample,  # type: ignore
        kwargs_list=ps_args,
        init_fn=_init_dataset,
        init_args=(dset_cls, dset_kwargs),
        pre_fn=None,
        post_fn=None,
        n_workers=n_workers,
        n_proc_per_worker=n_proc_per_worker,
        seed=seed,
        logfile=os.path.join(processed_path, "keys.txt"),
    )

    failed_keys = sorted(
        [get_data_args[i] for i in range(len(get_data_args)) if not completeds[i]]
    )
    if not all(completeds):
        ks = "\n".join([",".join(key) for key in failed_keys])
        print(f"failed items: {ks}")
        raise ValueError("Sampling failed, please debug.")


T = TypeVar("T", bound=CanSample)


class CachedByKeyDataset(tgd.Dataset, Generic[T]):
    """A hierarchical dataset, where each key has multiple data samples associated with it.

    Consider each key as a unique entity; for instance, the entity might be:
        * A single object
        * A scene, with different settings

    Each entity may be sampled; this sampling may be deterministic, or may be stochastic,
    depending on the setting. For instance, if we're sampling an object which has some internal
    state, sampling multiple times with the same key may yield different results.

    Given a set of entities, we'd like to do normal dataset operation things (i.e. repeat, iterate,
    random access, etc.). However, we'd also like to be able to generate an offline dataset as well,
    for performance reasons. So we'd like to be able to sample on-the-fly, or read from
    memory. In the in-memory configuration, each entity gets its own SinglePathDataset.
    """

    def __init__(
        self,
        dset_cls: Type[T],
        dset_kwargs: Dict[str, Any],
        data_keys: Union[Sequence[Tuple[str, ...]], Sequence[str]],
        root: Union[str, Path],
        processed_dirname: str,
        n_repeat: int,
        n_workers: int = -1,
        n_proc_per_worker: int = 2,
        seed=None,
        transform: Optional[Callable] = None,
        pre_transform: Optional[Callable] = None,
        pre_filter: Optional[Callable] = None,
        log: bool = True,
    ):
        data_keys = [(key,) if isinstance(key, str) else key for key in data_keys]
        for key in data_keys:
            for arg in key:
                if not isinstance(arg, str):
                    raise ValueError("sample keys have to be strings, for now...")
        # Dataset.
        self._dset_cls = dset_cls
        self._dset_kwargs = dset_kwargs
        self._data_keys: Sequence[Tuple[str, ...]] = data_keys

        # Sampling parameters.
        self._processed_dirname = processed_dirname
        self._n_repeat = n_repeat
        self._n_workers = n_workers
        self._n_proc_per_worker = n_proc_per_worker

        # Random seed. Important for reproducibility!
        self._seed = seed

        self.dataset = self._dset_cls(**dset_kwargs)

        # This has to come before inmem_dset is created.
        super().__init__(root, transform, pre_transform, pre_filter, log)

        # Mapping from key to dataset.
        self.inmem_dsets: Dict[Tuple[str, ...], SinglePathDataset] = {
            key: SinglePathDataset(data_path)
            for key, data_path in zip(self._data_keys, self.processed_paths)
        }

        # Concatenated dataset.
        self.concat_dset: td.ConcatDataset = td.ConcatDataset(
            [self.inmem_dsets[key] for key in self._data_keys]
        )

    @property
    def processed_file_names(self) -> List[str]:
        return [f"{'_'.join(key)}_{self._n_repeat}.pt" for key in self._data_keys]

    @property
    def processed_dir(self) -> str:
        return os.path.join(self.root, self._processed_dirname)

    def len(self) -> int:
        return len(self._data_keys) * self._n_repeat

    def get(self, idx: int):
        return self.concat_dset[idx]

    def get_data(self, *args, idx: int = 0):
        return self.inmem_dsets[args][idx]

    def process(self):
        parallel_sample(
            self._dset_cls,
            self._dset_kwargs,
            self._data_keys,
            self.processed_dir,
            self._n_repeat,
            n_workers=self._n_workers,
            n_proc_per_worker=self._n_proc_per_worker,
            seed=self._seed,
        )
