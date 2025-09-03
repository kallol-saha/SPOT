from torch.utils.data import DataLoader
import numpy as np

import pytorch_lightning as pl
from placement_suggester.dataset.ndf_dataset import JointOccDemoDataset

class NDFDemoDataModule(pl.LightningDataModule):
    def __init__(self,
                 dataset_root='/media/jenny/cubbins-archive/jwang_datasets/ndf_robot/src/ndf_robot/data/demos/mug/grasp_rim_hang_handle_gaussian_precise_w_shelf/',
                 # TODO make this different from dataset_root maybe
                 test_dataset_root='/media/jenny/cubbins-archive/jwang_datasets/ndf_robot/src/ndf_robot/data/demos/mug/grasp_rim_hang_handle_gaussian_precise_w_shelf/',
                 n_demos=0, # 0 means all demos
                 demo_placement_surface="rack",
                 load_shelf=False,
                 anchor_cls="rack",
                 overfit=False,
                 num_overfit_transforms=3,
                 seed_overfit_transforms=False,
                 set_Y_transform_to_identity=False,
                 set_Y_transform_to_overfit=False,
                 batch_size=8,
                 num_workers=8,
                 rotation_variance=np.pi/180 * 5,
                 translation_variance=0.1,
                 num_points=1024,
                 synthetic_occlusion=False,
                 ball_radius=None,
                 plane_occlusion=False,
                 ball_occlusion=False,
                 plane_standoff=None,
                 occlusion_class=2,
                 dataset_size=1000,
                 symmetric_class=None, # unused
                 ):
        super().__init__()
        self.dataset_root = dataset_root
        self.test_dataset_root = test_dataset_root
        self.batch_size = batch_size
        self.num_workers = num_workers

        self.n_demos = n_demos
        self.demo_placement_surface = demo_placement_surface
        self.load_shelf = load_shelf
        self.anchor_cls = anchor_cls

        self.overfit = overfit
        self.num_overfit_transforms = num_overfit_transforms
        self.seed_overfit_transforms = seed_overfit_transforms
        # identity has a higher priority than overfit
        self.set_Y_transform_to_identity = set_Y_transform_to_identity
        self.set_Y_transform_to_overfit = set_Y_transform_to_overfit
        if self.set_Y_transform_to_identity:
            self.set_Y_transform_to_overfit = True
        self.rotation_variance = rotation_variance
        self.translation_variance = translation_variance
        self.num_points=num_points
        self.synthetic_occlusion = synthetic_occlusion
        self.ball_radius = ball_radius
        self.plane_occlusion = plane_occlusion
        self.ball_occlusion = ball_occlusion
        self.plane_standoff = plane_standoff
        self.occlusion_class = occlusion_class
        self.dataset_size = dataset_size
        # self.symmetric_class = symmetric_class

    def pass_loss(self, loss):
        self.loss = loss.to(self.device)

    def prepare_data(self):
        '''called only once and on 1 GPU'''
        pass

    def setup(self, stage=None):
        '''called one each GPU separately - stage defines if we are at fit or test step'''
        # we set up only relevant datasets when stage is specified (automatically set by Pytorch-Lightning)
        if stage == 'fit' or stage is None:
            print("TRAIN Dataset")
            print(self.dataset_root)
            self.train_dataset = JointOccDemoDataset(
                # Variables from NDF dataset starter code
                demo_log_dir=self.dataset_root, 
                n_demos=self.n_demos, # 0 means all demos 
                demo_placement_surface=self.demo_placement_surface, 
                load_shelf=self.load_shelf,

                # Additional settings
                anchor_cls=self.anchor_cls,

                # Variables from point_cloud_dataset.py
                overfit=self.overfit,
                num_overfit_transforms=self.num_overfit_transforms,
                seed_overfit_transforms=self.seed_overfit_transforms,
                set_Y_transform_to_identity=self.set_Y_transform_to_identity,
                set_Y_transform_to_overfit=self.set_Y_transform_to_overfit,
                rotation_variance=self.rotation_variance,
                translation_variance=self.translation_variance,
                num_points=self.num_points,
                synthetic_occlusion=self.synthetic_occlusion,
                ball_radius=self.ball_radius,
                plane_occlusion=self.plane_occlusion,
                ball_occlusion=self.ball_occlusion,
                plane_standoff=self.plane_standoff,
                occlusion_class=self.occlusion_class,
                dataset_size=self.dataset_size,
                # symmetric_class=self.symmetric_class,
            )
        if stage == 'val' or stage is None:
            print("VAL Dataset")
            self.val_dataset = JointOccDemoDataset(
                # Variables from NDF dataset starter code
                demo_log_dir=self.test_dataset_root, 
                n_demos=self.n_demos, # 0 means all demos 
                demo_placement_surface=self.demo_placement_surface, 
                load_shelf=self.load_shelf,

                # Additional settings
                anchor_cls=self.anchor_cls,

                # Variables from point_cloud_dataset.py
                overfit=self.overfit,
                num_overfit_transforms=self.num_overfit_transforms,
                seed_overfit_transforms=self.seed_overfit_transforms,
                set_Y_transform_to_identity=self.set_Y_transform_to_identity,
                set_Y_transform_to_overfit=self.set_Y_transform_to_overfit,
                rotation_variance=self.rotation_variance,
                translation_variance=self.translation_variance,
                num_points=self.num_points,
                synthetic_occlusion=self.synthetic_occlusion,
                ball_radius=self.ball_radius,
                plane_occlusion=self.plane_occlusion,
                ball_occlusion=self.ball_occlusion,
                plane_standoff=self.plane_standoff,
                occlusion_class=self.occlusion_class,
                dataset_size=self.dataset_size,
                # symmetric_class=self.symmetric_class,
            )
        if stage == 'test':
            print("TEST Dataset")
            raise ValueError("Test dataset not implemented yet")
            # self.test_dataset = TestPointCloudDataset(
            #     dataset_root=self.test_dataset_root,
            #     dataset_indices=self.dataset_index,  # [self.dataset_index],
            #     action_class=self.action_class,
            #     anchor_class=self.anchor_class,
            #     dataset_size=self.dataset_size,
            #     rotation_variance=self.rotation_variance,
            #     translation_variance=self.translation_variance,
            #     cloud_type=self.cloud_type,
            #     symmetric_class=self.symmetric_class,
            #     num_points=self.num_points,
            #     overfit=self.overfit,
            #     num_overfit_transforms=self.num_overfit_transforms,
            #     seed_overfit_transforms=self.seed_overfit_transforms,
            #     set_Y_transform_to_identity=self.set_Y_transform_to_identity,
            #     set_Y_transform_to_overfit=self.set_Y_transform_to_overfit,
            #     gripper_lr_label=self.gripper_lr_label,
            #     index_list=self.index_list,
            #     no_transform_applied=self.no_transform_applied,
            #     init_distribution_tranform_file=self.init_distribution_tranform_file,
            # )

    def return_index_list_test(self):
        return self.test_dataset.return_index_list()

    def train_dataloader(self):
        return DataLoader(self.train_dataset,
                          batch_size=self.batch_size,
                          num_workers=self.num_workers)

    def val_dataloader(self):
        return DataLoader(self.val_dataset,
                          batch_size=self.batch_size,
                          num_workers=self.num_workers)

    def test_dataloader(self):
        return DataLoader(self.test_dataset,
                          batch_size=self.batch_size,
                          num_workers=self.num_workers)
