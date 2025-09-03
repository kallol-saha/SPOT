import os
import tempfile

import numpy as np
import torch_geometric.data as tgd

from rpad.pyg.dataset import CachedByKeyDataset


class SimpleDataset:
    def __init__(self):
        super().__init__()

    def get_data(self, index, seed=None):
        rng = np.random.default_rng(seed)
        return tgd.Data(id=index, pos=rng.random((200, 3)))


def test_parallel_sample():
    with tempfile.TemporaryDirectory() as tmpdir:
        dset = CachedByKeyDataset(
            dset_cls=SimpleDataset,
            dset_kwargs=dict(),
            data_keys=[(str(i),) for i in range(100)],
            root=tmpdir,
            processed_dirname="processed_test",
            n_repeat=10,
            n_workers=5,
            seed=12345,
        )

        # First, try to get some properties.
        assert len(dset) == 1000

        # Then try to get some data.
        data = dset[0]

        assert data.id == "0"
        assert len(data.pos) == 200

        # Then make sure everything is there.
        # There are 3 extra files that are generated.
        assert len(os.listdir(os.path.join(tmpdir, "processed_test"))) == 103
        for i in range(100):
            assert os.path.exists(os.path.join(tmpdir, "processed_test", f"{i}_10.pt"))


def test_parallel_reproducible():
    with tempfile.TemporaryDirectory() as tmpdir:
        dset = CachedByKeyDataset(
            dset_cls=SimpleDataset,
            dset_kwargs=dict(),
            data_keys=[(str(i),) for i in range(100)],
            root=tmpdir,
            processed_dirname="processed_test",
            n_repeat=10,
            n_workers=0,
            seed=12345,
        )

        # Then try to get some data.
        data1 = dset[0]

    with tempfile.TemporaryDirectory() as tmpdir:
        dset = CachedByKeyDataset(
            dset_cls=SimpleDataset,
            dset_kwargs=dict(),
            data_keys=[(str(i),) for i in range(100)],
            root=tmpdir,
            processed_dirname="processed_test",
            n_repeat=10,
            n_workers=0,
            seed=12345,
        )

        # Then try to get some data.
        data2 = dset[0]

    assert np.array_equal(data1.pos, data2.pos)

    with tempfile.TemporaryDirectory() as tmpdir:
        dset = CachedByKeyDataset(
            dset_cls=SimpleDataset,
            dset_kwargs=dict(),
            data_keys=[(str(i),) for i in range(100)],
            root=tmpdir,
            processed_dirname="processed_test",
            n_repeat=10,
            n_workers=0,
            seed=54321,
        )

        # Then try to get some data.
        data3 = dset[0]

    # Different seed -> different pos.
    assert not np.array_equal(data1.pos, data3.pos)
