from placement_suggester.utils.se3 import random_se3
from pytorch3d.transforms import Rotate
from placement_suggester.utils.occlusion_utils import ball_occlusion, plane_occlusion

import numpy as np
import torch
from torch.utils.data import Dataset
import random
import glob
import os
import os.path as osp
from scipy.spatial.transform import Rotation
import pickle
from placement_suggester.utils import ndf_geometry
from pytorch3d.ops import sample_farthest_points
from pathlib import Path

from ndf_robot.utils import util
from ndf_robot.utils.eval_gen_utils import (
    soft_grasp_close, constraint_grasp_close, constraint_obj_world, constraint_grasp_open, 
    safeCollisionFilterPair, object_is_still_grasped, get_ee_offset, post_process_grasp_point,
    process_demo_data_rack, process_demo_data_shelf, process_xq_data, process_xq_rs_data, safeRemoveConstraint,
)


class JointOccTrainDataset(Dataset):
    def __init__(self, ndf_data_path=None, sidelength=None, depth_aug=False, multiview_aug=False, phase='train', obj_class='all', train_num=None):

        self.ndf_data_path = Path(ndf_data_path)
        # Path setup (change to folder where your training data is kept)
        # these are the names of the full dataset folders
        mug_path = osp.join(
            self.ndf_data_path, 'training_data/mug_table_all_pose_4_cam_half_occ_full_rand_scale')
        bottle_path = osp.join(
            self.ndf_data_path, 'training_data/bottle_table_all_pose_4_cam_half_occ_full_rand_scale')
        bowl_path = osp.join(
            self.ndf_data_path, 'training_data/bowl_table_all_pose_4_cam_half_occ_full_rand_scale')

        # these are the names of the mini-dataset folders, to ensure everything is up and running
        # mug_path = osp.join(self.ndf_data_path, 'training_data/test_mug')
        # bottle_path = osp.join(self.ndf_data_path, 'training_data/test_bottle')
        # bowl_path = osp.join(self.ndf_data_path, 'training_data/test_bowl')

        if obj_class == 'all' or (type(obj_class) is list and obj_class[0] == 'all'):
            paths = [mug_path, bottle_path, bowl_path]
        else:
            paths = []
            if 'mug' in obj_class:
                paths.append(mug_path)
            if 'bowl' in obj_class:
                paths.append(bowl_path)
            if 'bottle' in obj_class:
                paths.append(bottle_path)
        if len(paths) == 0:
            print("WARNING: No matching object types match obj_class")
        # print('Loading from paths: ', paths)

        files_total = []
        for path in paths:

            files = list(sorted(glob.glob(path+"/*.npz")))
            n = len(files)

            if train_num == None:
                idx = int(0.9 * n)
            else:
                idx = train_num

            if phase == 'train':
                files = files[:idx]
            else:
                files = files[idx:]

            files_total.extend(files)

        self.files = files_total

        self.sidelength = sidelength
        self.depth_aug = depth_aug
        self.multiview_aug = multiview_aug

        block = 128
        bs = 1 / block
        hbs = bs * 0.5
        self.bs = bs
        self.hbs = hbs

        self.shapenet_mug_dict = pickle.load(open(
            osp.join(self.ndf_data_path, 'training_data/occ_shapenet_mug.p'), 'rb'))
        self.shapenet_bowl_dict = pickle.load(open(
            osp.join(self.ndf_data_path, 'training_data/occ_shapenet_bowl.p'), "rb"))
        self.shapenet_bottle_dict = pickle.load(open(osp.join(
            self.ndf_data_path, 'training_data/occ_shapenet_bottle.p'), "rb"))

        self.shapenet_dict = {'03797390': self.shapenet_mug_dict,
                              '02880940': self.shapenet_bowl_dict, '02876657': self.shapenet_bottle_dict}

        self.projection_mode = "perspective"

        self.cache_file = None
        self.count = 0

        print("files length ", len(self.files))
        
        self.num_points = 1024

    def __len__(self):
        return len(self.files)

    def get_item(self, index):
        try:
            data = np.load(self.files[index], allow_pickle=True)
            # legacy naming, used to use pose expressed in camera frame. global reference frame doesn't matter though
            posecam = data['object_pose_cam_frame']

            idxs = list(range(posecam.shape[0]))
            random.shuffle(idxs)
            select = random.randint(1, 4)

            if self.multiview_aug:
                idxs = idxs[:select]

            poses = []
            quats = []
            for i in idxs:
                pos = posecam[i, :3]
                quat = posecam[i, 3:]

                poses.append(pos)
                quats.append(quat)

            shapenet_id = str(data['shapenet_id'].item())
            category_id = str(data['shapenet_category_id'].item())

            depths = []
            segs = []
            cam_poses = []

            for i in idxs:
                seg = data['object_segmentation'][i, 0]
                depth = data['depth_observation'][i]

                rix = np.random.permutation(depth.shape[0])[:1000]
                seg = seg[rix]
                depth = depth[rix]

                if self.depth_aug:
                    depth = depth + np.random.randn(*depth.shape) * 0.1

                segs.append(seg)
                depths.append(torch.from_numpy(depth))
                cam_poses.append(data['cam_pose_world'][i])

            # change these values depending on the intrinsic parameters of camera used to collect the data. These are what we used in pybullet
            y, x = torch.meshgrid(torch.arange(480), torch.arange(640))

            # Compute native intrinsic matrix
            sensor_half_width = 320
            sensor_half_height = 240

            vert_fov = 60 * np.pi / 180

            vert_f = sensor_half_height / np.tan(vert_fov / 2)
            hor_f = sensor_half_width / (np.tan(vert_fov / 2) * 320 / 240)

            intrinsics = np.array(
                [[hor_f, 0., sensor_half_width, 0.],
                 [0., vert_f, sensor_half_height, 0.],
                 [0., 0., 1., 0.]]
            )

            # Rescale to new sidelength
            intrinsics = torch.from_numpy(intrinsics)

            # build depth images from data
            dp_nps = []
            for i in range(len(segs)):
                seg_mask = segs[i]
                dp_np = ndf_geometry.lift(x.flatten()[seg_mask], y.flatten()[
                    seg_mask], depths[i].flatten(), intrinsics[None, :, :])
                dp_np = torch.cat(
                    [dp_np, torch.ones_like(dp_np[..., :1])], dim=-1)
                dp_nps.append(dp_np)

            # load in voxel occupancy data
            voxel_path = osp.join(category_id, shapenet_id,
                                  'models', 'model_normalized_128.mat')
            coord, voxel_bool, _ = self.shapenet_dict[category_id][voxel_path]

            rix = np.random.permutation(coord.shape[0])

            coord = coord[rix[:1500]]
            label = voxel_bool[rix[:1500]]

            offset = np.random.uniform(-self.hbs, self.hbs, coord.shape)
            coord = coord + offset
            coord = coord * data['mesh_scale']

            coord = torch.from_numpy(coord)

            # transform everything into the same frame
            transforms = []
            for quat, pos in zip(quats, poses):
                quat_list = [float(quat[0]), float(quat[1]),
                             float(quat[2]), float(quat[3])]
                rotation_matrix = Rotation.from_quat(quat_list)
                rotation_matrix = rotation_matrix.as_matrix()

                transform = np.eye(4)
                transform[:3, :3] = rotation_matrix
                transform[:3, -1] = pos
                transform = torch.from_numpy(transform)
                transforms.append(transform)

            transform = transforms[0]
            coord = torch.cat([coord, torch.ones_like(coord[..., :1])], dim=-1)
            coord = torch.sum(transform[None, :, :]
                              * coord[:, None, :], dim=-1)
            coord = coord[..., :3]

            points_world = []

            for i, dp_np in enumerate(dp_nps):
                point_transform = torch.matmul(
                    transform, torch.inverse(transforms[i]))
                dp_np = torch.sum(
                    point_transform[None, :, :] * dp_np[:, None, :], dim=-1)
                points_world.append(dp_np[..., :3])

            point_cloud = torch.cat(points_world, dim=0)

            rix = torch.randperm(point_cloud.size(0))
            point_cloud = point_cloud[rix[:self.num_points]]

            # point_cloud, _ = sample_farthest_points(point_cloud.unsqueeze(0),
            #                                         K=1000, random_start_point=True)

            if point_cloud.size(0) != self.num_points:
                return self.get_item(index=random.randint(0, self.__len__() - 1))

            label = (label - 0.5) * 2.0

            # translate everything to the origin based on the point cloud mean
            center = point_cloud.mean(dim=0)
            coord = coord - center[None, :]
            point_cloud = point_cloud - center[None, :]

            labels = label

            # at the end we have 3D point cloud observation from depth images, voxel occupancy values and corresponding voxel coordinates
            res = {'point_cloud': point_cloud.float(),
                   'coords': coord.float(),
                   'intrinsics': intrinsics.float(),
                   'cam_poses': np.zeros(1)}  # cam poses not used
            return res

        except Exception as e:
            print(e)
        #    print(file)
            return self.get_item(index=random.randint(0, self.__len__() - 1))

    def __getitem__(self, index):
        return self.get_item(index)


class JointOccDemoDataset(Dataset):
    def __init__(
            self, 

            # Variables from NDF dataset starter code
            demo_log_dir, 
            n_demos=0, # 0 means all demos 
            demo_placement_surface="rack", 
            load_shelf=False,

            # Additional settings
            anchor_cls="rack",

            # Variables from point_cloud_dataset.py
            overfit=False,
            num_overfit_transforms=3,
            seed_overfit_transforms=False,
            set_Y_transform_to_identity=False,
            set_Y_transform_to_overfit=False,
            rotation_variance=np.pi,
            translation_variance=0.5,
            num_points=1024,
            synthetic_occlusion=False,
            ball_radius=None,
            plane_occlusion=False,
            ball_occlusion=False,
            plane_standoff=None,
            occlusion_class=2,
            symmetric_class=None,
            dataset_size=1000,
        ):
        # Example dataset location: /media/jenny/cubbins-archive/jwang_datasets/ndf_robot/src/ndf_robot/data/demos/mug/grasp_rim_hang_handle_gaussian_precise_w_shelf/
        
        assert anchor_cls in ["rack", "gripper"]
        self.anchor_cls = anchor_cls

        # Defining some variables for the copied NDF script to work
        self.global_dict = {
            'demo_load_dir': demo_log_dir
        }
        self.n_demos = n_demos
        self.demo_placement_surface = demo_placement_surface
        self.load_shelf = load_shelf
        assert self.demo_placement_surface in ["shelf", "rack"]

        # Variables from point_cloud_dataset.py
        self.overfit = overfit
        self.seed_overfit_transforms = seed_overfit_transforms
        # identity has a higher priority than overfit
        self.set_Y_transform_to_identity = set_Y_transform_to_identity
        self.set_Y_transform_to_overfit = set_Y_transform_to_overfit
        if self.set_Y_transform_to_identity:
            self.set_Y_transform_to_overfit = True
        self.num_overfit_transforms = num_overfit_transforms
        self.T0_list = []
        self.T1_list = []
        self.T2_list = []
        self.rot_var = rotation_variance
        self.trans_var = translation_variance
        self.num_points = num_points
        self.synthetic_occlusion = synthetic_occlusion
        self.ball_radius = ball_radius
        self.plane_occlusion = plane_occlusion
        self.ball_occlusion = ball_occlusion
        self.plane_standoff = plane_standoff
        self.occlusion_class = occlusion_class
        self.symmetric_class = symmetric_class
        self.dataset_size = dataset_size

        self.load_filenames()
        
        if self.overfit or self.set_Y_transform_to_overfit:
            self.get_fixed_transforms()

    """
    From point_cloud_dataset.py
    """
    def get_fixed_transforms(self):
        # Copied from point_cloud_dataset.py
        points_action, points_anchor = self.load_data(
            index=0,
        )
        if self.overfit:
            if self.seed_overfit_transforms:
                torch.random.manual_seed(0)
            for i in range(self.num_overfit_transforms):
                a = random_se3(1, rot_var=self.rot_var,
                               trans_var=self.trans_var, device=points_action.device)
                b = random_se3(1, rot_var=self.rot_var,
                               trans_var=self.trans_var, device=points_anchor.device)
                if self.set_Y_transform_to_identity:
                    c = Rotate(torch.eye(3), device=points_anchor.device)
                else:
                    c = random_se3(1, rot_var=self.rot_var,
                                   trans_var=self.trans_var, device=points_anchor.device)
                self.T0_list.append(a)
                self.T1_list.append(b)
                self.T2_list.append(c)
            print(f"\nOverfitting transform lists in PointCloudDataset:\n\t-TO: {[m.get_matrix() for m in self.T0_list]}\n\t-T1: {[m.get_matrix() for m in self.T1_list]}\n\t-T2: {[m.get_matrix() for m in self.T2_list]}")
        elif self.set_Y_transform_to_overfit:
            if self.seed_overfit_transforms:
                torch.random.manual_seed(0)
            for i in range(self.num_overfit_transforms):
                if self.set_Y_transform_to_identity:
                    c = Rotate(torch.eye(3), device=points_anchor.device)
                else:
                    c = random_se3(1, rot_var=self.rot_var,
                                   trans_var=self.trans_var, device=points_anchor.device)
                self.T2_list.append(c)
        return

    def load_filenames(self):
        # Starter code from https://github.com/anthonysimeonov/ndf_robot/blob/master/src/ndf_robot/eval/evaluate_ndf.py#L121

        # get filenames of all the demo files
        demo_filenames = os.listdir(self.global_dict['demo_load_dir'])
        assert len(demo_filenames), 'No demonstrations found in path: %s!' % self.global_dict['demo_load_dir']

        # strip the filenames to properly pair up each demo file
        grasp_demo_filenames_orig = [osp.join(self.global_dict['demo_load_dir'], fn) for fn in demo_filenames if 'grasp_demo' in fn]  # use the grasp names as a reference

        self.place_demo_filenames = []
        self.grasp_demo_filenames = []
        for i, fname in enumerate(grasp_demo_filenames_orig):
            shapenet_id_npz = fname.split('/')[-1].split('grasp_demo_')[-1]
            place_fname = osp.join('/'.join(fname.split('/')[:-1]), 'place_demo_' + shapenet_id_npz)
            if osp.exists(place_fname):
                self.grasp_demo_filenames.append(fname)
                self.place_demo_filenames.append(place_fname)
            else:
                print('Could not find corresponding placement demo: %s, skipping ' % place_fname)

        success_list = []
        place_success_list = []
        place_success_teleport_list = []
        grasp_success_list = []

        demo_shapenet_ids = []

        # get info from all demonstrations
        demo_target_info_list = []
        demo_rack_target_info_list = []

        if self.n_demos > 0:
            gp_fns = list(zip(self.grasp_demo_filenames, self.place_demo_filenames))
            gp_fns = random.sample(gp_fns, self.n_demos)
            self.grasp_demo_filenames, self.place_demo_filenames = zip(*gp_fns)
            self.grasp_demo_filenames, self.place_demo_filenames = list(self.grasp_demo_filenames), list(self.place_demo_filenames)
            print('USING ONLY %d DEMONSTRATIONS' % len(self.grasp_demo_filenames))
            print(self.grasp_demo_filenames, self.place_demo_filenames)
        else:
            print('USING ALL %d DEMONSTRATIONS' % len(self.grasp_demo_filenames))

    def __len__(self):
        return self.dataset_size
    
    def process_data_like_point_cloud_dataset(self, points_action, points_anchor, index): #, symmetric_cls, index):
        # Copied from point_cloud_dataset.py

        # from pytorch3d.transforms import Transform3d
        if self.overfit:
            transform_idx = torch.randint(
                self.num_overfit_transforms, (1,)).item()
            T0 = self.T0_list[transform_idx]
            T1 = self.T1_list[transform_idx]
            if not self.set_Y_transform_to_overfit:
                T2 = random_se3(1, rot_var=self.rot_var,
                                trans_var=self.trans_var, device=points_anchor.device)
            else:
                T2 = self.T2_list[transform_idx]
        else:
            T0 = random_se3(1, rot_var=self.rot_var,
                            trans_var=self.trans_var, device=points_action.device)
            T1 = random_se3(1, rot_var=self.rot_var,
                            trans_var=self.trans_var, device=points_anchor.device)
            if self.set_Y_transform_to_identity:
                T2 = Rotate(torch.eye(3), device=points_anchor.device)
            elif self.set_Y_transform_to_overfit:
                transform_idx = torch.randint(
                        self.num_overfit_transforms, (1,)).item()
                T2 = self.T2_list[transform_idx]
            else:
                T2 = random_se3(1, rot_var=self.rot_var,
                                trans_var=self.trans_var, device=points_anchor.device)

        if(points_action.shape[1] > self.num_points):
            if self.synthetic_occlusion and self.action_class == self.occlusion_class:
                if self.ball_occlusion:
                    points_action = ball_occlusion(
                        points_action[0], radius=self.ball_radius).unsqueeze(0)
                if self.plane_occlusion:
                    points_action = plane_occlusion(
                        points_action[0], stand_off=self.plane_standoff).unsqueeze(0)

            if(points_action.shape[1] > self.num_points):
                # TODO PUT THIS RANDOMNESS BACK
                points_action, action_ids = sample_farthest_points(points_action,
                                                                   K=self.num_points, random_start_point=False)
                                                                   # K=self.num_points, random_start_point=True)
            elif(points_action.shape[1] < self.num_points):
                raise NotImplementedError(
                    f'Action point cloud is smaller than cloud size ({points_action.shape[1]} < {self.num_points})')

            # if(len(symmetric_cls) > 0):
            #     symmetric_cls = symmetric_cls[action_ids.view(-1)]
        elif(points_action.shape[1] < self.num_points):
            raise NotImplementedError(
                f'Action point cloud is smaller than cloud size ({points_action.shape[1]} < {self.num_points})')

        if(points_anchor.shape[1] > self.num_points):
            if self.synthetic_occlusion and self.anchor_class == self.occlusion_class:
                if self.ball_occlusion:
                    points_anchor = ball_occlusion(
                        points_anchor[0], radius=self.ball_radius).unsqueeze(0)
                if self.plane_occlusion:
                    points_anchor = plane_occlusion(
                        points_anchor[0], stand_off=self.plane_standoff).unsqueeze(0)
            if(points_anchor.shape[1] > self.num_points):
                # TODO PUT THIS RANDOMNESS BACK
                points_anchor, _ = sample_farthest_points(points_anchor,
                                                          K=self.num_points, random_start_point=False)
                                                          # K=self.num_points, random_start_point=True)
            elif(points_anchor.shape[1] < self.num_points):
                raise NotImplementedError(
                    f'Anchor point cloud is smaller than cloud size ({points_anchor.shape[1]} < {self.num_points})')
        elif(points_anchor.shape[1] < self.num_points):
            raise NotImplementedError(
                f'Anchor point cloud is smaller than cloud size ({points_anchor.shape[1]} < {self.num_points})')

        points_action_trans = T0.transform_points(points_action)
        points_anchor_trans = T1.transform_points(points_anchor)

        points_action_onetrans = T2.transform_points(points_action)
        points_anchor_onetrans = T2.transform_points(points_anchor)

        # if self.symmetric_class is not None:
        #     symmetric_cls = self.get_sym_label(
        #         action_cloud=points_action, anchor_cloud=points_anchor, action_class=self.action_class, anchor_class=self.anchor_class)  # num_points, 1
        #     symmetric_cls = symmetric_cls.unsqueeze(-1)
        #     if self.action_class == 0:

        #         points_action = torch.cat(
        #             [points_action, symmetric_cls], axis=-1)

        #         points_anchor = torch.cat(
        #             [points_anchor, torch.ones(symmetric_cls.shape)], axis=-1)
        #         points_action_trans = torch.cat(
        #             [points_action_trans, symmetric_cls], axis=-1)
        #         points_anchor_trans = torch.cat(
        #             [points_anchor_trans, torch.ones(symmetric_cls.shape)], axis=-1)

        #     elif self.anchor_class == 0:

        #         points_anchor = torch.cat(
        #             [points_anchor, symmetric_cls], axis=-1)

        #         points_action = torch.cat(
        #             [points_action, torch.ones(symmetric_cls.shape)], axis=-1)
        #         points_anchor_trans = torch.cat(
        #             [points_anchor_trans, symmetric_cls], axis=-1)
        #         points_action_trans = torch.cat(
        #             [points_action_trans, torch.ones(symmetric_cls.shape)], axis=-1)

        data = {
            'points_action': points_action.squeeze(0),
            'points_anchor': points_anchor.squeeze(0),
            'points_action_trans': points_action_trans.squeeze(0),
            'points_anchor_trans': points_anchor_trans.squeeze(0),
            'points_action_onetrans': points_action_onetrans.squeeze(0),
            'points_anchor_onetrans': points_anchor_onetrans.squeeze(0),
            'T0': T0.get_matrix().squeeze(0),
            'T1': T1.get_matrix().squeeze(0),
            'T2': T2.get_matrix().squeeze(0),
            # 'symmetric_cls': symmetric_cls,
            'mug_id': self.place_demo_filenames[index],
        }
        return data
    
    def load_data(self, index):
        # Defining some variables for the copied script to work
        max_bb_volume = 0
        place_xq_demo_idx = 0
        grasp_data_list = []
        place_data_list = []
        demo_rel_mat_list = []

        # load all the demo data and look at objects to help decide on query points
        grasp_demo_fn = self.grasp_demo_filenames[index]
        place_demo_fn = self.place_demo_filenames[index]
        grasp_data = np.load(grasp_demo_fn, allow_pickle=True)
        place_data = np.load(place_demo_fn, allow_pickle=True)

        grasp_data_list.append(grasp_data)
        place_data_list.append(place_data)

        start_ee_pose = grasp_data['ee_pose_world'].tolist()
        end_ee_pose = place_data['ee_pose_world'].tolist()
        place_rel_mat = util.get_transform(
            pose_frame_target=util.list2pose_stamped(end_ee_pose),
            pose_frame_source=util.list2pose_stamped(start_ee_pose)
        )
        place_rel_mat = util.matrix_from_pose(place_rel_mat)
        demo_rel_mat_list.append(place_rel_mat)

        optimizer_gripper_pts, rack_optimizer_gripper_pts, shelf_optimizer_gripper_pts = process_xq_data(grasp_data, place_data, shelf=self.load_shelf)
        optimizer_gripper_pts_rs, rack_optimizer_gripper_pts_rs, shelf_optimizer_gripper_pts_rs = process_xq_rs_data(grasp_data, place_data, shelf=self.load_shelf)

        if self.demo_placement_surface == 'shelf':
            # print('Using shelf points')
            place_optimizer_pts = shelf_optimizer_gripper_pts
            place_optimizer_pts_rs = shelf_optimizer_gripper_pts_rs
        elif self.demo_placement_surface == "rack":
            # print('Using rack points')
            place_optimizer_pts = rack_optimizer_gripper_pts
            place_optimizer_pts_rs = rack_optimizer_gripper_pts_rs
        else:
            raise ValueError(f"Invalid demo placement surface {self.demo_placement_surface}")

        if self.demo_placement_surface == 'shelf':
            target_info, rack_target_info, shapenet_id = process_demo_data_shelf(grasp_data, place_data, cfg=None)
        elif self.demo_placement_surface == 'rack':
            target_info, rack_target_info, shapenet_id = process_demo_data_rack(grasp_data, place_data, cfg=None)
        else:
            raise ValueError(f"Invalid demo placement surface {self.demo_placement_surface}")

        if self.demo_placement_surface == 'shelf':
            rack_target_info['demo_query_pts'] = place_optimizer_pts

        if self.anchor_cls == "rack":
            points_action = torch.tensor(rack_target_info['demo_obj_pts'])[None].float()
            points_anchor = torch.tensor(rack_target_info['demo_query_pts_real_shape'])[None].float()
            # symmetric_cls = None # TODO should symmetric_cls be None here?
        elif self.anchor_cls == "gripper":
            points_anchor = torch.tensor(target_info['demo_query_pts_real_shape'])[None].float()
            points_anchor = torch.tensor(target_info['demo_query_pts_real_shape'])[None].float()
            # symmetric_cls = None # TODO should symmetric_cls be None here?
        else:
            raise ValueError(f"Invalide anchor_cls {self.anchor_cls}")

        return points_action, points_anchor #, symmetric_cls

    def get_item(self, index):
        try:
            points_action, points_anchor = self.load_data(index)
            # TODO duplicate racks
            data = self.process_data_like_point_cloud_dataset(points_action, points_anchor, index)
            return data
        except Exception as e:
            print(e)
            return self.get_item(index=random.randint(0, self.__len__() - 1))

    def __getitem__(self, index):
        return self.get_item(index)
