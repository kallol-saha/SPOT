"""
This file contains the pybullet wrapper for the scene generation and object storage.
"""

import os
import time
from copy import deepcopy

import cv2
import numpy as np
import pybullet as p
import pybullet_data
import pybullet_utils.bullet_client as bc
import yaml
from omegaconf import OmegaConf
from scipy.spatial.transform import Rotation as R
from spot.submodules.pybullet_planning.pybullet_tools.ikfast.franka_panda.ik import (
    PANDA_INFO,
)
from spot.submodules.pybullet_planning.pybullet_tools.ikfast.ikfast import (
    check_ik_solver,
    either_inverse_kinematics,
)
from spot.submodules.pybullet_planning.pybullet_tools.utils import link_from_name

urdf_root_path = pybullet_data.getDataPath()
# Default start state is self-colliding, which causes motion planning to fail
ROBOT_START_STATE = np.array([0, 0, 0, -1, 0, 1.8, 0])
REL_GRASP_POSE_OFFSET = 0.1
DROP_HEIGHT = 0.05
MOVEMENT_HEIGHT = 0.8
GRASPS_TO_TRY = [  # Quaternion is x, y, z, w
    # Vertical grasps
    [0.0, 0.0, REL_GRASP_POSE_OFFSET, 0.707, -0.707, 0.0, 0.0],
    [0.0, 0.0, REL_GRASP_POSE_OFFSET, 1.0, 0.0, 0.0, 0.0],
    # Horizontal grasps
    # [0.0, REL_GRASP_POSE_OFFSET, 0.0, 0.5, 0.5, -0.5, 0.5],
    # [0.0, -REL_GRASP_POSE_OFFSET, 0.0, -0.5, -0.5, -0.5, 0.5],
    # [REL_GRASP_POSE_OFFSET, 0.0, 0.0, 0.707, 0.0, -0.707, 0.0],
    # [-REL_GRASP_POSE_OFFSET, 0.0, 0.0, 0.707, 0.0, 0.707, 0.0],
]
COLORS = {
    "blue": np.array([78, 121, 167]) / 255.0,  # blue
    "green": np.array([89, 161, 79]) / 255.0,  # green
    "brown": np.array([156, 117, 95]) / 255.0,  # brown
    "orange": np.array([242, 142, 43]) / 255.0,  # orange
    "yellow": np.array([237, 201, 72]) / 255.0,  # yellow
    "gray": np.array([186, 176, 172]) / 255.0,  # gray
    "red": np.array([255, 87, 89]) / 255.0,  # red
    "purple": np.array([176, 122, 161]) / 255.0,  # purple
    "cyan": np.array([118, 183, 178]) / 255.0,  # cyan
    "pink": np.array([255, 157, 167]) / 255.0,  # pink
}


class NoIKSolutionsException(Exception):
    pass


class CollisionException(Exception):
    pass


class ObjectNotMovedException(Exception):
    pass


def get_joint_id(body_id, joint_index=-1):
    """Return the joint id used in the pybullet segmentation
    given the body id and the joint index"""
    joint_index = (joint_index + 1) << 24
    return body_id + joint_index


class Object:
    """
    Wrapper for the objects in the scene.
    This class is used to store the objects in the scene and get their joint, segmentation id and position.
    """

    def __init__(self, id, joint, name, client_id, parent_object=None, is_fixed=False):
        self.id = id
        self.joint = joint
        self.seg = get_joint_id(id, joint)
        self.name = name
        self.client_id = client_id
        self.parent_object = parent_object
        self.is_fixed = is_fixed

    def __repr__(self) -> str:
        return f"{self.name} \n Id: {self.id} \n Joint: {self.joint} \n Segment id: {self.seg} \n"

    def get_pose(self):
        if self.joint == -1:
            return self.get_body_pos_and_orn()
        else:
            return self.get_link_pos_and_orn()

    def get_body_pos_and_orn(self):
        pos, orn = self.client_id.getBasePositionAndOrientation(self.id)
        return [pos[0], pos[1], pos[2], orn[0], orn[1], orn[2], orn[3]]

    def get_link_pos_and_orn(self):
        pos, orn = self.client_id.getLinkState(self.id, self.joint)[:2]
        return [pos[0], pos[1], pos[2], orn[0], orn[1], orn[2], orn[3]]

    def get_joint_value(self):
        if self.joint == -1:
            return None
        return [self.client_id.getJointState(self.id, self.joint)[0]]


class Camera:
    def __init__(self, cam_id, client_id, cam_args, record=False):
        self.client_id = client_id
        self.cam_id = cam_id
        self.cam_args = cam_args

        self.window = "Camera_" + str(self.cam_id)

        self.mode = cam_args["mode"]  # or "position"
        self.target = cam_args["target"]

        # For distance mode:
        self.distance = cam_args["distance"]
        self.yaw = cam_args["yaw"]
        self.pitch = cam_args["pitch"]
        self.roll = cam_args["roll"]
        self.up_axis_index = cam_args["up_axis_index"]

        # For position mode:
        self.eye = cam_args["eye"]
        self.up_vec = cam_args["up_vec"]

        # Intrinsics:
        self.width = cam_args["width"]
        self.height = cam_args["height"]
        self.fov = cam_args["fov"]
        self.near = cam_args["near"]
        self.far = cam_args["far"]

        # If camera is already saved somewhere:
        self.view_matrix = cam_args["view_matrix"]
        self.projection_matrix = cam_args["projection_matrix"]

        self.quat = R.from_euler("xyz", [self.yaw, self.pitch, self.roll]).as_quat()
        self.aspect = self.width / self.height

        if (self.view_matrix is None) or (self.view_matrix == "None"):
            if self.mode == "distance":
                self.view_matrix = self.client_id.computeViewMatrixFromYawPitchRoll(
                    self.target,
                    self.distance,
                    self.yaw,
                    self.pitch,
                    self.roll,
                    self.up_axis_index,
                )
            elif self.mode == "position":
                self.view_matrix = self.client_id.computeViewMatrix(
                    self.eye, self.target, self.up_vec
                )

        if (self.projection_matrix is None) or (self.projection_matrix == "None"):
            self.projection_matrix = self.client_id.computeProjectionMatrixFOV(
                self.fov, self.aspect, self.near, self.far
            )

    def capture(self):
        _, _, rgb, depth, segs = self.client_id.getCameraImage(
            self.width,
            self.height,
            self.view_matrix,
            self.projection_matrix,
        )

        # rgb = np.reshape(rgb, (self.width, self.height, 4))
        # rgb = rgb[..., :3]

        # depth = np.reshape(depth, (self.width, self.height))
        rgb = np.reshape(rgb, (self.height, self.width, 4))
        rgb = rgb[..., :3]

        depth = np.reshape(depth, (self.height, self.width))

        return rgb, depth, segs

    def get_pointcloud(self, depth, seg_img=None):
        """Returns a point cloud and its segmentation from the given depth image

        Args:
        -----
            depth (np.array): depth image
            width (int): width of the image
            height (int): height of the image
            view_matrix (np.array): 4x4 view matrix
            proj_matrix (np.array): 4x4 projection matrix
            seg_img (np.array): segmentation image

        Return:
        -------
            pcd (np.array): Nx3 point cloud
            pcd_seg (np.array): N array, segmentation of the point cloud
        """
        # based on https://stackoverflow.com/questions/59128880/getting-world-coordinates-from-opengl-depth-buffer

        # create a 4x4 transform matrix that goes from pixel coordinates (and depth values) to world coordinates
        proj_matrix = np.asarray(self.projection_matrix).reshape([4, 4], order="F")
        view_matrix = np.asarray(self.view_matrix).reshape([4, 4], order="F")
        tran_pix_world = np.linalg.inv(
            np.matmul(proj_matrix, view_matrix)
        )  # Pixel to 3D transformation

        # create a mesh grid with pixel coordinates, by converting 0 to width and 0 to height to -1 to 1
        y, x = np.mgrid[-1 : 1 : 2 / self.height, -1 : 1 : 2 / self.width]
        y *= -1.0  # y is reversed in pixel coordinates

        # Reshape to single dimension arrays
        x, y, z = x.reshape(-1), y.reshape(-1), depth.reshape(-1)

        # Homogenize:
        pixels = np.stack([x, y, z, np.ones_like(z)], axis=1)
        # filter out "infinite" depths:
        fin_depths = np.where(z < 0.99)
        pixels = pixels[fin_depths]

        # Depth z is between 0 to 1, so convert it to -1 to 1.
        pixels[:, 2] = 2 * pixels[:, 2] - 1

        if seg_img is not None:
            seg_img = np.array(seg_img)
            pcd_seg = seg_img.reshape(-1)[fin_depths]  # filter out "infinite" depths
        else:
            pcd_seg = None

        # turn pixels to world coordinates
        points = np.matmul(tran_pix_world, pixels.T).T
        points /= points[:, 3:4]  # Homogenize in 3D
        points = points[:, :3]  # Remove last axis ones

        return points, pcd_seg, fin_depths

    def move_camera(self):
        self.yaw = max(self.yaw + self.yaw_rate, self.yaw_limit)

        # Update view matrix:
        self.view_matrix = self.client_id.computeViewMatrixFromYawPitchRoll(
            self.target,
            self.distance,
            self.yaw,
            self.pitch,
            self.roll,
            self.up_axis_index,
        )

        # When limit is reached:
        if self.yaw == self.yaw_limit:
            # Exchange yaw limit and initial yaw
            self.yaw_limit, self.initial_yaw = self.initial_yaw, self.yaw_limit
            # Move in the other direction
            self.yaw_rate = -self.yaw_rate

    def record(self):
        rgb, _, _ = self.capture()
        rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)

        self.out.write(rgb)

        # Move the camera:
        self.move_camera()

    def start_recording(self, plan_folder):
        self.fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.out = cv2.VideoWriter(
            os.path.join(plan_folder, self.cam_args["output_file"]),
            self.fourcc,
            self.cam_args["fps"],
            (self.width, self.height),
        )

        self.moving = self.cam_args["moving"]
        self.yaw_limit = self.cam_args["yaw_limit"]
        self.yaw_rate = self.cam_args["yaw_rate"]
        self.initial_yaw = self.yaw

    def stop_recording(self):
        self.out.release()


class Scene:
    def __init__(
        self,
        args,
        seed=None,
        gui=True,
        robot=True,
        timestep=1 / 480,
        max_control_iters=1000,
        stability_iters=1000,
        tol=0.001,
    ):
        self.seed = seed
        self.args = args
        self.robot: bool = robot

        self.max_control_iters = max_control_iters
        self.stability_iters = stability_iters
        self.tol = tol

        config_path = os.path.join(
            self.args.scene_config_folder, self.args.scene + ".yaml"
        )
        assert os.path.isfile(
            config_path
        ), f"Error: {config_path} is not a file or does not exist! Check your configs"

        self.generate_scene(gui, config_path, timestep)

        # Setup camera
        self.camera_list = []
        for i, cam_args in self.config["cameras"].items():
            cam = Camera(int(i), self.client_id, cam_args)
            self.camera_list.append(cam)

        if self.args.recording_camera is not None:
            self.record = True
            record_cam_args = OmegaConf.to_container(
                self.args.recording_camera, resolve=True
            )
            self.recording_cam = Camera(
                -1,  # Recording camera is given id -1
                client_id=self.client_id,
                cam_args=record_cam_args,
                record=True,
            )
        else:
            self.record = False

        print("Loading Perception Modules")
        self.prev_press = -1
        self.num_pressed = 1
        self.current_focus = 0

        self.prev_keys = {}

    def get(self, object_name):
        return self.__getattribute__(object_name)

    def get_all_objects(self):
        return [self.__getattribute__(obj) for obj in self.objects]

    def generate_scene(self, gui, config_path, timestep):
        self.client_id = bc.BulletClient(
            p.GUI if gui else p.DIRECT
        )  # Initialize the bullet client

        self.client_id.setAdditionalSearchPath(
            pybullet_data.getDataPath()
        )  # Add pybullet's data package to path
        self.client_id.setTimeStep(timestep)  # Set simulation timestep
        self.client_id.configureDebugVisualizer(
            p.COV_ENABLE_SHADOWS, 1
        )  # Enable Shadows
        self.client_id.configureDebugVisualizer(
            p.COV_ENABLE_GUI, 0
        )  # Disable Frame Axes

        self.client_id.resetSimulation()
        self.client_id.setGravity(0, 0, -9.8)  # Set Gravity

        self.plane = self.client_id.loadURDF(
            "plane.urdf", basePosition=(0, 0, 0), useFixedBase=True
        )  # Load a floor

        with open(config_path, "r") as file:
            self.config = yaml.load(file, Loader=yaml.FullLoader)

        self.load_objects()

        # Save the scene state
        self.initial_state = self.client_id.saveState()

    def reset(self):
        self.client_id.restoreState(stateId=self.initial_state)

    def load_robot(self):
        assert (
            "robot" in self.config
        ), "Error: A robot key does not exist in the config file"

        robot_info = self.config["robot"]
        robot_path = robot_info["file"]
        self.robot_id = self.client_id.loadURDF(
            robot_path,
            robot_info["pos"],
            robot_info["orn"],
            useFixedBase=robot_info["fixed_base"],
            globalScaling=robot_info["scale"],
        )

        self.joints = []
        self.gripper_joints = []

        for i in range(self.client_id.getNumJoints(self.robot_id)):
            info = self.client_id.getJointInfo(self.robot_id, i)

            joint_id = info[0]
            joint_name = info[1].decode("utf-8")
            joint_type = info[2]

            if joint_name == "panda_grasptarget_hand":
                self.end_effector = joint_id

            if (
                joint_type == p.JOINT_REVOLUTE
            ):  # 0-6 are revolute, 7-8 rigid, 9-10 prismatic, 11 rigid
                self.joints.append(joint_id)

            if joint_type == p.JOINT_PRISMATIC:
                self.gripper_joints.append(joint_id)

        self.joint_lower_limits = np.array(
            [
                -166 * (np.pi / 180),
                -101 * (np.pi / 180),
                -166 * (np.pi / 180),
                -176 * (np.pi / 180),
                -166 * (np.pi / 180),
                -1 * (np.pi / 180),
                -166 * (np.pi / 180),
            ]
        )

        self.joint_upper_limits = np.array(
            [
                166 * (np.pi / 180),
                101 * (np.pi / 180),
                166 * (np.pi / 180),
                -4 * (np.pi / 180),
                166 * (np.pi / 180),
                215 * (np.pi / 180),
                166 * (np.pi / 180),
            ]
        )

        self.gripper_lower_limits = np.array([1e-6, 1e-6])
        self.gripper_upper_limits = np.array([0.039, 0.039])
        self.grasp_depth = robot_info["grasp_depth"]

        self.upper_limit = np.append(self.joint_upper_limits, self.gripper_upper_limits)
        self.lower_limit = np.append(self.joint_lower_limits, self.gripper_lower_limits)
        self.joint_range = self.upper_limit - self.lower_limit
        self.rest_pose = np.zeros((9,))

        self.end_effector = link_from_name(
            self.robot_id, "tool_link"
        )  # This is just the link ID (a number)

        self.left_finger = link_from_name(
            self.robot_id, "panda_leftfinger"
        )  # This is just the link ID (a number)

        self.right_finger = link_from_name(
            self.robot_id, "panda_rightfinger"
        )  # This is just the link ID (a number)

        # Increase friction of fingers to be able to grip objects
        self.client_id.changeDynamics(
            self.robot_id, self.left_finger, lateralFriction=1000.0
        )
        self.client_id.changeDynamics(
            self.robot_id, self.right_finger, lateralFriction=1000.0
        )
        # Increase friction between blocks
        for _id in self.grasp_obj_ids:
            self.client_id.changeDynamics(_id, -1, lateralFriction=0.4)

        self.work_ratio = np.array(robot_info["joint_work_ratio"])
        self.is_grasped = False
        self.drops = 0

        # Creating a gear constraint to keep the fingers centered according to (https://github.com/bulletphysics/bullet3/issues/2101#issuecomment-574087554)
        self.c = self.client_id.createConstraint(
            self.robot_id,
            9,
            self.robot_id,
            10,
            jointType=self.client_id.JOINT_GEAR,
            jointAxis=[1, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=[0, 0, 0],
        )
        self.client_id.changeConstraint(self.c, gearRatio=-1, erp=0.1, maxForce=50)

        return Object(
            id=self.robot_id,
            joint=-1,
            name="robot",
            client_id=self.client_id,
            is_fixed=True,
        )

    def load_objects(self):
        assert "objects" in self.config, "Error: No objects in the config file"

        self.objects = {}
        self.object_grasps = {}
        self.grasp_obj_names = []
        self.grasp_obj_ids = []
        self.fixed_obj_ids = []

        loaded_objects = {}
        for obj_name, obj in self.config["objects"].items():
            obj_path = os.path.join(self.args.objects_folder, obj["file"])
            obj_id = self.client_id.loadURDF(
                obj_path,
                obj["pos"],
                obj["orn"],
                useFixedBase=obj["fixed_base"],
                globalScaling=obj["scale"],
            )

            loaded_objects[obj_name] = Object(
                id=obj_id,
                joint=-1,
                name=obj_name,
                client_id=self.client_id,
                is_fixed=obj["fixed_base"],
            )

            # Add the color to the objects
            if "color" in obj:
                self.client_id.changeVisualShape(
                    obj_id,
                    -1,
                    rgbaColor=obj["color"],
                )
                loaded_objects[obj_name].color = obj["color"]

            # For demo collection
            self.objects[obj_name] = obj_id
            if not obj["fixed_base"]:
                self.object_grasps[obj_name] = obj["grasp"]
                self.grasp_obj_names.append(obj_name)
                self.grasp_obj_ids.append(obj_id)
                self.client_id.changeDynamics(obj_id, -1, lateralFriction=0.2)
            else:
                self.fixed_obj_ids.append(obj_id)

        if self.robot:
            robot_obj = self.load_robot()
            loaded_objects["robot"] = robot_obj

        for obj in loaded_objects.values():
            setattr(self, obj.name, obj)

        self.movable_objects = [
            obj.name for obj in loaded_objects.values() if obj.is_fixed == False
        ]

        self.num_objs = len(self.object_grasps)
        self.controlled_obj = 0
        self.controlled_obj_name = self.grasp_obj_names[self.controlled_obj]

        # print("Currently controlling: " + self.controlled_obj_name)

    def set_initial_state(self, data):
        # Assumes `load_objects` has already been called
        for obj_id, pose in data.items():
            self.client_id.resetBasePositionAndOrientation(obj_id, pose[:3], pose[3:])

        self.initial_state = self.client_id.saveState()

    def transform_object(self, obj_name, transform):
        transform_pose = self.transformation_to_pose(transform)
        curr_pose = self.get_object_pose(obj_name)

        pose = self.combine_poses([transform_pose, curr_pose])

        self.client_id.resetBasePositionAndOrientation(
            self.objects[obj_name], pose[:3], pose[3:]
        )

    def move_object(
        self,
        state,
        obj_name,
        transform,
        teleport: bool = False,
        fail_on_not_moved: bool = False,
    ):
        """Move object according to the transform."""
        self.client_id.restoreState(state)
        obj_id = self.objects[obj_name]

        if teleport:
            # "Teleportation" dynamics: teleport the object to the proposed transform
            # then wait for all objects to settle.
            self.transform_object(obj_name, transform)
            self.wait_for_stability()
        else:
            # Execute with a simulated robot using grasp heuristics and motion planning
            success = False

            # Try every possible grasp pose
            for relative_grasp_pose in GRASPS_TO_TRY:
                # Reset to a valid start state
                self.client_id.restoreState(state)

                # Object grasp pose
                obj_pose = self.get_object_pose(obj_name)
                grasp_pose = self.combine_poses([obj_pose, relative_grasp_pose])
                grasp_joints = self.inverse_kinematics(grasp_pose)

                # Find pose above grasp pose
                above_grasp_pose = grasp_pose.copy()
                above_grasp_pose[2] = MOVEMENT_HEIGHT
                above_grasp_joints = self.inverse_kinematics(above_grasp_pose)

                # Get dropping pose
                transform_pose = self.transformation_to_pose(transform)
                obj_drop_pose = self.combine_poses([transform_pose, obj_pose])
                obj_drop_pose[2] += DROP_HEIGHT  # Drop from slightly above
                drop_pose = above_grasp_pose.copy()
                drop_pose[:3] = obj_drop_pose[:3]
                drop_joints = self.inverse_kinematics(drop_pose)

                # Find pose above drop pose
                above_drop_pose = drop_pose.copy()
                above_drop_pose[2] = MOVEMENT_HEIGHT
                above_drop_joints = self.inverse_kinematics(above_drop_pose)

                # ---- CHECK GRASP ---- #

                state_before_check = self.client_id.saveState()

                # Teleport gripper to starting position
                self.set_joint_pos(grasp_joints)
                self.set_gripper_pos(self.gripper_upper_limits)
                curr_pose = self.get_end_effector_pose()
                # Ending grasp pose
                grasp_action = np.zeros((7,))
                grasp_action[2] = self.grasp_depth
                grasp_action[-1] = 1
                grasp_end_pose = self.combine_poses([curr_pose, grasp_action])
                grasp_end_joints = self.inverse_kinematics(grasp_end_pose)
                self.set_joint_pos(grasp_end_joints)

                # Check if the manipulator is in collision, then revert back to initial state and run again:
                collision, _ = self.is_robot_in_collision()
                self.client_id.restoreState(stateId=state_before_check)
                if collision:
                    continue  # Try next grasp pose

                # ---- EXECUTION ---- #

                self.go_to_position_interpolated(above_grasp_joints, num=50)
                self.go_to_position_interpolated(grasp_joints, num=20)

                try:
                    self.grasp(obj_id)
                except (CollisionException, NoIKSolutionsException):
                    print("Failed to grasp")
                    continue  # Try next grasp pose

                self.go_to_position_interpolated(above_grasp_joints, num=20)
                self.go_to_position_interpolated(above_drop_joints, num=50)
                self.go_to_position_interpolated(drop_joints, num=20)

                self.drop()

                self.go_to_position_interpolated(above_drop_joints, num=20)

                # ------------------- #

                time.sleep(0.1)
                success = True
                break

            if not success:
                self.client_id.restoreState(state)
                if fail_on_not_moved:
                    raise ObjectNotMovedException
                print(f"[WARN] Object {obj_name} was not moved!")

        pcd, pcd_seg, rgb = self.get_observation()
        state = self.client_id.saveState()
        rgb_img, _, _ = self.camera_list[0].capture()

        # TODO: Check here if everything is within bounds, or return a fail flag
        return pcd, pcd_seg, rgb, state, rgb_img

    def move_object_transform(self, state, obj_name: str, transform):
        """
        Move the object by the desired transform but do NOT step the simulator.

        AKA "TaxPose-D dynamics." Objects may be levitating.
        """
        self.client_id.restoreState(state)

        self.transform_object(obj_name, transform)

        pcd, pcd_seg, rgb = self.get_observation()
        state = self.client_id.saveState()
        rgb_img, _, _ = self.camera_list[0].capture()
        return pcd, pcd_seg, rgb, state, rgb_img

    def heuristic(self, state):
        self.client_id.restoreState(state)

        # Z positions of R, G and B:        TODO: Change this to top, middle, bottom
        R = self.get_object_pose("red_cube")
        B = self.get_object_pose("blue_cube")
        G = self.get_object_pose("green_cube")

        h = 3

        # Goal is RBG from top to bottom:
        if 0.63 < G[2] < 0.67:
            # This means green cube is on the table
            h -= 1

            if (G[2] + 0.03) < B[2] < (G[2] + 0.07) and 0.03 < np.linalg.norm(
                (B[:3] - G[:3])
            ) < 0.07:
                # Also, blue is on green
                h -= 1

                if (B[2] + 0.03) < R[2] < (B[2] + 0.07) and 0.03 < np.linalg.norm(
                    (R[:3] - B[:3])
                ) < 0.07:
                    # Additionally, red is on top of blue
                    h -= 1

        # Goal is BRG from top to bottom:
        # if 0.63 < G[2] < 0.67:
        #     # This means green cube is on the table
        #     h -= 1

        #     if (G[2] + 0.03) < R[2] < (G[2] + 0.07) and 0.03 < np.linalg.norm(
        #         R[:3] - G[:3]
        #     ) < 0.07:
        #         # Also, red is on green
        #         h -= 1

        #         if (R[2] + 0.03) < B[2] < (R[2] + 0.07) and 0.03 < np.linalg.norm(
        #             B[:3] - R[:3]
        #         ) < 0.07:
        #             # Additionally, blue is on top of red
        #             h -= 1

        return h

    def get_observation(self):
        # TODO: this is duplicated code with spot.demo.KeyboardDemo.save_data
        pcd, pcd_seg, rgb = self.get_fused_pcd()

        # Remove background and robot
        mask = pcd_seg != 0
        if self.robot:
            mask = mask & (pcd_seg != self.robot_id)

        # !!! I am hardcoding here to crop the table !!!
        table_mask = ((np.abs(pcd[:, 0]) < 0.2) & (np.abs(pcd[:, 1]) < 0.2)) | (
            pcd_seg != self.objects["table"]
        )
        mask = table_mask & mask
        # !!! End of hard-coding !!!

        pcd = pcd[mask]
        pcd_seg = pcd_seg[mask]
        rgb = rgb[mask]

        # !!! I am hardcoding here to remove points from the table !!!
        n = 5
        table_indices = np.where(pcd_seg == self.objects["table"])[0]
        keep_indices = table_indices[::n]
        mask = np.ones(pcd_seg.shape, dtype=bool)
        mask[table_indices] = False
        mask[keep_indices] = True
        pcd = pcd[mask]
        pcd_seg = pcd_seg[mask]
        rgb = rgb[mask]
        # !!! End of hard-coding !!!

        return pcd, pcd_seg, rgb

    def control_objects(self):
        self.controlled_obj_name = self.grasp_obj_names[self.controlled_obj]
        pose = self.get_object_pose(self.controlled_obj_name)
        pos = pose[:3]
        euler = np.array(p.getEulerFromQuaternion(pose[3:]))

        keys = self.client_id.getKeyboardEvents()

        left, right = p.B3G_LEFT_ARROW, p.B3G_RIGHT_ARROW
        up, down = p.B3G_UP_ARROW, p.B3G_DOWN_ARROW
        front, back = ord("-"), ord("=")

        roll_in, roll_out = ord("["), ord("]")
        pitch_in, pitch_out = ord(";"), ord("'")
        yaw_in, yaw_out = ord("."), ord(",")

        focus = ord("/")
        drop = p.B3G_RETURN

        step = 0.00001
        angle_step = 0.00001

        # Positive X
        if front in keys:
            pos[0] = pos[0] + step
        # Negative X
        if back in keys:
            pos[0] = pos[0] - step

        # Positive Y
        if left in keys:
            pos[1] = pos[1] + step
        # Negative Y
        if right in keys:
            pos[1] = pos[1] - step

        # Positive Z
        if up in keys:
            pos[2] = pos[2] + step
        # Negative Z
        if down in keys:
            pos[2] = pos[2] - step

        # Roll:
        if roll_out in keys:
            euler[0] = euler[0] + angle_step
        if roll_in in keys:
            euler[0] = euler[0] - angle_step

        # Pitch
        if pitch_out in keys:
            euler[1] = euler[1] + angle_step
        if pitch_in in keys:
            euler[1] = euler[1] - angle_step

        # Yaw
        if yaw_out in keys:
            euler[2] = euler[2] + angle_step
        if yaw_in in keys:
            euler[2] = euler[2] - angle_step

        # Switch Focus
        if (focus in self.prev_keys) and (len(keys) == 0):
            self.controlled_obj = (self.controlled_obj + 1) % self.num_objs
            self.controlled_obj_name = self.grasp_obj_names[self.controlled_obj]
            # print("Currently controlling: " + self.controlled_obj_name)
            self.prev_keys = keys.copy()
            return 0

        # Drop object:
        if (drop in self.prev_keys) and (len(keys) == 0):
            self.wait_for_stability()
            self.prev_keys = keys.copy()
            return self.objects[self.controlled_obj_name]

        self.prev_keys = keys.copy()

        orn = p.getQuaternionFromEuler(euler)
        self.client_id.resetBasePositionAndOrientation(
            self.objects[self.controlled_obj_name], pos, orn
        )

        return 0

    def control_view(self):
        view_cam = self.client_id.getDebugVisualizerCamera()
        yaw, pitch, dist, target = (
            view_cam[8],
            view_cam[9],
            view_cam[10],
            np.array(view_cam[11]),
        )

        keys = self.client_id.getKeyboardEvents()
        left, right = p.B3G_LEFT_ARROW, p.B3G_RIGHT_ARROW
        up, down = p.B3G_UP_ARROW, p.B3G_DOWN_ARROW
        zoom_in, zoom_out = ord("."), ord(",")
        focus = ord("/")

        if (len(keys) > 0) and (self.prev_press == p.KEY_IS_DOWN):
            self.num_pressed += 1
        else:
            self.num_pressed = 1

        # Yaw Left
        if (left in keys) and (
            keys[left] == p.KEY_IS_DOWN or keys[left] == p.KEY_WAS_TRIGGERED
        ):
            yaw = yaw - 0.1 * self.num_pressed
            self.prev_press = keys[left]
        # Yaw Right
        if (right in keys) and (
            keys[right] == p.KEY_IS_DOWN or keys[right] == p.KEY_WAS_TRIGGERED
        ):
            yaw = yaw + 0.1 * self.num_pressed
            self.prev_press = keys[right]
        # Pitch Up
        if (up in keys) and (
            keys[up] == p.KEY_IS_DOWN or keys[up] == p.KEY_WAS_TRIGGERED
        ):
            pitch = pitch - 0.1 * self.num_pressed
            self.prev_press = keys[up]
        # Pitch Down
        if (down in keys) and (
            keys[down] == p.KEY_IS_DOWN or keys[down] == p.KEY_WAS_TRIGGERED
        ):
            pitch = pitch + 0.1 * self.num_pressed
            self.prev_press = keys[down]

        # Zoom in:
        if (zoom_in in keys) and (
            keys[zoom_in] == p.KEY_IS_DOWN or keys[zoom_in] == p.KEY_WAS_TRIGGERED
        ):
            dist = dist - 0.01 * self.num_pressed
            self.prev_press = keys[zoom_in]
        # Zoom out:
        if (zoom_out in keys) and (
            keys[zoom_out] == p.KEY_IS_DOWN or keys[zoom_out] == p.KEY_WAS_TRIGGERED
        ):
            dist = dist + 0.01 * self.num_pressed
            self.prev_press = keys[zoom_out]

        # Switch Focus
        if (focus in keys) and (keys[focus] == p.KEY_IS_DOWN):
            self.current_focus = (self.current_focus + 1) % self.num_objs
            self.prev_press = keys[focus]

        target = self.get_object_pose(self.grasp_obj_names[self.current_focus])[:3]

        self.client_id.resetDebugVisualizerCamera(dist, yaw, pitch, target)

    def go_to_position(
        self, joints, detect_collisions: bool = False, slow: bool = False
    ):
        # gripper_pos = self.get_gripper_pos()

        # if self.is_grasped:
        #     target_gripper_pos = self.gripper_lower_limits
        # else:
        #     target_gripper_pos = gripper_pos.copy()

        # # Keep gripper stable while moving
        # self.client_id.setJointMotorControlArray(
        #     self.robot_id,
        #     jointIndices=self.gripper_joints,
        #     controlMode=p.POSITION_CONTROL,
        #     targetPositions=target_gripper_pos,
        #     forces=[5.0, 5.0],  # Max grasp force
        # )

        self.client_id.setJointMotorControlArray(
            self.robot_id,
            jointIndices=self.joints,
            controlMode=p.POSITION_CONTROL,
            targetPositions=joints,
        )

        for _ in range(self.max_control_iters):
            self.client_id.stepSimulation()
            if slow:
                time.sleep(0.01)
            if (
                detect_collisions
                and len(self.client_id.getContactPoints(self.robot_id)) > 0
            ):
                raise CollisionException

            joint_pos = self.get_joint_pos()
            error = np.abs(joints - joint_pos)
            if np.all(error < self.tol):
                break

    def is_robot_in_collision(self):
        self.client_id.stepSimulation()
        contacts = self.client_id.getContactPoints(self.robot_id)

        return (len(contacts) > 0), contacts

    def execute_path(self, path, detect_collisions: bool = False, slow=False):
        if type(path) == list:
            path_len = len(path)
        else:
            path_len = path.shape[0]
        for i in range(path_len):
            self.go_to_position(path[i], detect_collisions=detect_collisions, slow=slow)
            if self.record:
                self.recording_cam.record()

    def actuate_gripper(self, gripper_joints):
        self.client_id.setJointMotorControlArray(
            self.robot_id,
            jointIndices=self.gripper_joints,
            controlMode=p.POSITION_CONTROL,
            targetPositions=gripper_joints,
            forces=[5.0, 5.0],  # Max grasp force
        )

        for i in range(self.max_control_iters):
            self.client_id.stepSimulation()
            if self.record and (i % self.args.actuate_gripper_record_skip == 0):
                self.recording_cam.record()
            gripper_pos = self.get_gripper_pos()
            error = np.abs(gripper_joints - gripper_pos)
            if np.all(error < self.tol):
                break

    def open_gripper(self):
        self.actuate_gripper(self.gripper_upper_limits)

    def close_gripper(self, obj_id):
        self.client_id.setJointMotorControlArray(
            self.robot_id,
            jointIndices=self.gripper_joints,
            controlMode=p.POSITION_CONTROL,
            targetPositions=self.gripper_lower_limits,
            forces=[5.0, 5.0],  # Max grasp force
        )

        for i in range(self.max_control_iters):
            self.client_id.stepSimulation()
            if self.record and (i % self.args.close_gripper_record_skip == 0):
                self.recording_cam.record()

            left_contact = (
                len(
                    self.client_id.getContactPoints(
                        self.robot_id, obj_id, self.left_finger, -1
                    )
                )
                > 0
            )
            right_contact = (
                len(
                    self.client_id.getContactPoints(
                        self.robot_id, obj_id, self.right_finger, -1
                    )
                )
                > 0
            )

            if left_contact and right_contact:
                # print("Contact detected")
                for _ in range(self.max_control_iters):
                    gripper_vel = self.get_gripper_vel()
                    error = np.abs(gripper_vel)
                    if np.all(error < self.tol):
                        break
                # print("Gripper stopped")
                break

    def stabilize_gripper(self):
        curr_gripper = self.get_gripper_pos()
        # self.client_id.setJointMotorControlArray(
        #     self.robot_id,
        #     jointIndices=self.gripper_joints,
        #     controlMode=p.VELOCITY_CONTROL,
        #     targetPositions=curr_gripper,
        #     targetVelocities=[0., 0.],
        #     forces=[.05, .05],  # Max grasp force
        # )

        self.client_id.resetJointState(
            self.robot_id, self.gripper_joints, curr_gripper, [0.0, 0.0]
        )

    def wait_for_stability(self):
        for k in range(self.stability_iters):
            self.client_id.stepSimulation()
            if self.record and (k % self.args.stability_record_skip == 0):
                self.recording_cam.record()
            obj_vels = np.zeros((self.num_objs, 6))
            for i in range(self.num_objs):
                obj_vels[i] = self.get_object_vel(self.grasp_obj_names[i])

            error = np.abs(obj_vels)
            if np.all(error < self.tol):
                break

    def get_joint_pos(self):
        states = self.client_id.getJointStates(self.robot_id, self.joints)
        pos = np.zeros(
            (
                len(
                    self.joints,
                )
            )
        )
        for i in range(len(states)):
            pos[i] = states[i][0]

        return np.array(pos)

    def set_joint_pos(self, joints):
        for i, joint_ind in enumerate(self.joints):
            self.client_id.resetJointState(self.robot_id, joint_ind, joints[i])

    def set_gripper_pos(self, gripper_joints):
        for i, joint_ind in enumerate(self.gripper_joints):
            self.client_id.resetJointState(self.robot_id, joint_ind, gripper_joints[i])

    def get_gripper_pos(self):
        states = self.client_id.getJointStates(self.robot_id, self.gripper_joints)
        pos = np.zeros(
            (
                len(
                    self.gripper_joints,
                )
            )
        )
        for i in range(len(states)):
            pos[i] = states[i][0]

        return np.array(pos)

    def get_gripper_vel(self):
        states = self.client_id.getJointStates(self.robot_id, self.gripper_joints)
        vel = np.zeros(
            (
                len(
                    self.gripper_joints,
                )
            )
        )
        for i in range(len(states)):
            vel[i] = states[i][1]

        return np.array(vel)

    def get_object_pose(self, obj_name: str):
        if type(obj_name) == int:  # If it is object id
            pose = self.client_id.getBasePositionAndOrientation(obj_name)
        else:
            pose = self.client_id.getBasePositionAndOrientation(self.objects[obj_name])
        return np.array([*pose[0], *pose[1]])

    def get_object_vel(self, obj_name):
        vel = self.client_id.getBaseVelocity(self.objects[obj_name])
        return np.array([*vel[0], *vel[1]])

    def combine_poses(self, pose_list):
        """
        Order of the list is the order in which it will be applied
        """

        T = np.eye(4)
        for pose in pose_list:
            T = T @ self.pose_to_transformation(pose)

        final_pose = self.transformation_to_pose(T)

        return final_pose

    def inverse_kinematics(self, pose):
        curr_joints = self.get_joint_pos()

        info = PANDA_INFO
        check_ik_solver(info)
        pose = (
            tuple(pose[:3]),
            tuple(pose[3:]),
        )  # Here quaternion has to be in x,y,z,w
        all_solns = np.array(
            list(
                either_inverse_kinematics(
                    self.robot_id,
                    info,
                    self.end_effector,
                    pose,
                    max_attempts=1000,
                    max_time=1000,
                    verbose=False,
                )
            )
        )
        if all_solns.size == 0:
            raise NoIKSolutionsException()

        error = np.max(
            np.abs(all_solns - curr_joints[np.newaxis, :])
            * self.work_ratio[np.newaxis, :],
            axis=1,
        )

        best_index = np.argmin(error)  # Take the best min(max()) score
        best_soln = all_solns[best_index]

        return best_soln

    def motion_planning_to_target_pose(self, target_pose: np.array):
        """Returns a planned path if both IK and motion planning succeed, None otherwise."""
        assert self.robot, "Cannot do motion planning without robot loaded"

        try:
            target_joints = self.inverse_kinematics(target_pose)
        except NoIKSolutionsException:
            print(f"*** No IK solution found for target pose {target_pose}!")
            return None

        res, path = self.ompl_interface.plan(target_joints)
        if res:
            return path
        else:
            print(f"*** Motion planning to target joints {target_joints} failed!")
            return None

    def auto_grasp_nearest(self):
        gripper_position = self.get_end_effector_pose()[
            np.newaxis, :3
        ]  # Expand so I can subtract later with equal dimensions
        obj_positions = np.zeros((self.num_objs, 3))

        for i in range(self.num_objs):
            obj_positions[i] = self.get_object_pose(self.grasp_obj_names[i])[
                :3
            ]  # This is in x,y,z,w

        error = np.sqrt(np.sum((obj_positions - gripper_position) ** 2, axis=1))
        nearest_obj_index = np.argmin(error)
        nearest_obj = self.grasp_obj_names[nearest_obj_index]

        obj_id = self.objects[
            nearest_obj
        ]  # For articulated objects, this has to be changed to a base link and a child link !!!
        object_pose = self.get_object_pose(nearest_obj)
        relative_grasp_pose = self.object_grasps[nearest_obj]

        # Initial grasp pose
        grasp_pose = self.combine_poses([object_pose, relative_grasp_pose])
        grasp_joints = self.inverse_kinematics(
            grasp_pose
        )  # This automatically gives the nearest IK solution to the current joint state

        tr = self.pose_to_transformation(grasp_pose)
        self.draw_frame(tr)

        self.go_to_position(grasp_joints)
        self.grasp(obj_id)

    def get_grasp_pose(self, obj):
        object_pose = self.get_object_pose(obj)
        relative_grasp_pose = self.object_grasps[obj]
        grasp_pose = self.combine_poses([object_pose, relative_grasp_pose])

        return grasp_pose

    def go_to_position_interpolated(
        self, joints, detect_collisions=False, slow=False, num=80
    ):
        curr_joints = self.get_joint_pos()
        grasp_path = np.linspace(curr_joints, joints, num=num)

        self.execute_path(grasp_path, detect_collisions=detect_collisions, slow=slow)

    def grasp(self, obj_id):
        """
        This function assumes that the robot is already at the grasp pose of the target object
        It will open the gripper, move forward, close the gripper, move back
        """
        curr_pose = self.get_end_effector_pose()

        # Open gripper:
        self.open_gripper()
        self.wait_for_stability()

        # Ending grasp pose
        grasp_action = np.zeros((7,))
        grasp_action[2] = self.grasp_depth
        grasp_action[-1] = 1
        grasp_end_pose = self.combine_poses([curr_pose, grasp_action])

        grasp_end_joints = self.inverse_kinematics(grasp_end_pose)

        # Actuate forward:
        curr_joints = self.get_joint_pos()
        self.go_to_position_interpolated(
            grasp_end_joints, detect_collisions=True, num=10
        )
        # curr_joints = self.get_joint_pos()
        # grasp_path = np.linspace(curr_joints, grasp_end_joints, num=80)

        # self.execute_path(grasp_path, detect_collisions=True)
        # time.sleep(0.1)

        self.close_gripper(obj_id)
        self.grasped_obj = obj_id
        self.is_grasped = True

        self.go_to_position_interpolated(curr_joints, num=10)
        # grasp_end_joints = self.get_joint_pos()
        # retract_path = np.linspace(grasp_end_joints, curr_joints, num=80)
        # self.execute_path(retract_path)

    def drop(self):
        if self.is_grasped:
            self.open_gripper()
            self.wait_for_stability()
            self.actuate_gripper(self.gripper_lower_limits)

            self.is_grasped = False
            self.drops += 1

    def save_img_and_seg(self):
        for i in range(len(self.camera_list)):
            cam = self.camera_list[i]
            cam_rgb, cam_depth = cam.capture()
            cam_pcd, cam_pcd_seg, cam_pcd_ind = cam.get_pointcloud(cam_depth)

            r = np.max(cam_depth) - np.min(cam_depth)
            m = np.min(cam_depth)
            cam_depth = np.round((255 / r) * (cam_depth - m)).astype(np.uint8)

            cv2.imwrite(
                "vis/Camera_" + str(i + 1) + "RGB_" + str(self.drops + 1) + ".jpg",
                cv2.cvtColor(cam_rgb, cv2.COLOR_BGR2RGB),
            )
            cv2.imwrite(
                "vis/Camera_" + str(i + 1) + "Depth_" + str(self.drops + 1) + ".jpg",
                cam_depth,
            )
            # cv2.imwrite("vis/Camera_" + str(i+1) + "Seg_" + str(self.drops+1) + ".jpg", cam_pcd_seg)

        pcd, rgb = self.get_fused_pcd()
        np.save("vis/pcd_" + str(self.drops) + ".npy", pcd)
        np.save("vis/pcd_rgb_" + str(self.drops) + ".npy", rgb)

    def get_fused_pcd(self):
        rgbs = []
        pcds = []
        pcd_segs = []

        for i in range(len(self.camera_list)):
            cam = self.camera_list[i]
            cam_rgb, cam_depth, cam_seg = cam.capture()
            cam_pcd, cam_pcd_seg, cam_pcd_ind = cam.get_pointcloud(cam_depth, cam_seg)
            rgbs.append(cam_rgb.reshape(-1, 3)[cam_pcd_ind])
            pcds.append(cam_pcd)
            pcd_segs.append(cam_pcd_seg)

        pcd = np.concatenate(pcds, axis=0)  # Fuse point clouds by simply stacking them
        rgb = np.concatenate(rgbs, axis=0)  # Optionally, get colors for each point
        pcd_segs = np.concatenate(pcd_segs, axis=0)

        return pcd, pcd_segs, rgb

    def get_end_effector_pose(self):
        pos, ori = self.client_id.getLinkState(
            self.robot_id, self.end_effector, computeForwardKinematics=1
        )[:2]
        pose = np.array([*pos, *ori])

        return pose

    def quaternion_to_rotation_matrix(self, quat):
        """
        Convert a quaternion to a rotation matrix.

        :param q: Quaternion [w, x, y, z]
        :return: 3x3 rotation matrix
        """
        # w, x, y, z = quat
        # rotation_matrix = np.array([[1 - 2*y**2 - 2*z**2,  2*x*y - 2*z*w,        2*x*z + 2*y*w],
        #                             [2*x*y + 2*z*w,        1 - 2*x**2 - 2*z**2,  2*y*z - 2*x*w],
        #                             [2*x*z - 2*y*w,        2*y*z + 2*x*w,        1 - 2*x**2 - 2*y**2]])

        mat = np.array(self.client_id.getMatrixFromQuaternion(quat))
        rotation_matrix = np.reshape(mat, (3, 3))

        return rotation_matrix

    def pose_to_transformation(self, pose):
        pos = pose[:3]
        quat = pose[3:]

        rotation_matrix = self.quaternion_to_rotation_matrix(quat)

        transform = np.zeros((4, 4))
        transform[:3, :3] = rotation_matrix.copy()
        transform[:3, 3] = pos.copy()
        transform[3, 3] = 1

        return transform

    def transformation_to_pose(self, T):
        trans = T[:3, 3]  # Extract translation (3x1 vector)
        rot = T[:3, :3]  # Extract rotation (3x3 matrix)
        quat = R.from_matrix(rot).as_quat()  # Convert to quaternion (w, x, y, z)

        pose = np.append(trans, quat)

        return pose

    def forward_kinematics(self, joint_angles):
        T_EE = np.identity(4)
        for i in range(7 + 3):
            T_EE = T_EE @ self.get_tf_mat(i, joint_angles)

        return T_EE

    def draw_frame(self, transform, scale_factor=0.2):
        unit_axes_world = np.array(
            [
                [scale_factor, 0, 0],
                [0, scale_factor, 0],
                [0, 0, scale_factor],
                [1, 1, 1],
            ]
        )
        axis_points = ((transform @ unit_axes_world)[:3, :]).T
        axis_center = transform[:3, 3]

        l1 = self.client_id.addUserDebugLine(
            axis_center, axis_points[0], COLORS["red"], lineWidth=4
        )
        l2 = self.client_id.addUserDebugLine(
            axis_center, axis_points[1], COLORS["green"], lineWidth=4
        )
        l3 = self.client_id.addUserDebugLine(
            axis_center, axis_points[2], COLORS["blue"], lineWidth=4
        )

        frame_id = [l1, l2, l3]

        return frame_id[:]

    def remove_frame(self, frame_id):
        for id in frame_id:
            self.client_id.removeUserDebugItem(id)
