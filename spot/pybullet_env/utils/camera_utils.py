import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as R

from spot.utils.pcd_utils import get_pointcloud


class Camera:
    def __init__(
        self,
        width=512,
        height=512,
        fov=60,
        near=0.02,
        far=50,
        target_position=[0, 0, 0],
        distance=1,
        yaw=0,
        pitch=0,
        roll=0,
        up_axis_index=2,
        view_matrix=None,
        projection_matrix=None,
    ):
        self.width = width
        self.height = height
        self.fov = fov
        self.near = near
        self.far = far
        self.target_position = target_position
        self.distance = distance
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll
        self.up_axis_index = up_axis_index

        self.quat = R.from_euler("xyz", [self.yaw, self.pitch, self.roll]).as_quat()
        self.aspect = self.width / self.height

        if view_matrix is None:
            self.view_matrix = p.computeViewMatrixFromYawPitchRoll(
                self.target_position,
                self.distance,
                self.yaw,
                self.pitch,
                self.roll,
                self.up_axis_index,
            )
        else:
            self.view_matrix = view_matrix

        if projection_matrix is None:
            self.projection_matrix = p.computeProjectionMatrixFOV(
                self.fov, self.aspect, self.near, self.far
            )
        else:
            self.projection_matrix = projection_matrix

    def get_rgb(self):
        img = p.getCameraImage(
            self.width,
            self.height,
            self.view_matrix,
            self.projection_matrix,
        )
        rgb = img[2]
        rgb = np.reshape(rgb, (self.width, self.height, 4))
        rgb = rgb[:, :, :3]
        return rgb

    def get_depth(self):
        img = p.getCameraImage(
            self.width,
            self.height,
            self.view_matrix,
            self.projection_matrix,
        )
        depth = img[3]
        depth = np.reshape(depth, (self.width, self.height))
        return depth


left_camera = Camera(
    target_position=[0, 0, 1.0],
    distance=1.1,
    yaw=-50,
    pitch=-30,
    roll=0,
    up_axis_index=2,
)

right_camera = Camera(
    target_position=[0, 0, 1.0],
    distance=1.1,
    yaw=50,
    pitch=-30,
    roll=0,
    up_axis_index=2,
)

camera_list = [left_camera, right_camera]


def get_pcd_from_sim(camera, sim=0):
    # Get observations
    images = p.getCameraImage(
        camera.width,
        camera.height,
        camera.view_matrix,
        camera.projection_matrix,
        shadow=1,
        renderer=p.ER_BULLET_HARDWARE_OPENGL,
        flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX,
        physicsClientId=sim,
    )

    depth = np.array(images[3]).reshape([camera.width, camera.height])
    seg = np.array(images[4]).reshape([camera.width, camera.height])

    pcd, pcd_seg = get_pointcloud(
        depth,
        camera.height,
        camera.width,
        camera.view_matrix,
        camera.projection_matrix,
        seg,
    )
    return pcd, pcd_seg


def in_arm_camera(robotId):
    fov, aspect, nearplane, farplane = 60, 1.0, 0.01, 100
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearplane, farplane)
    # Center of mass position and orientation (of link-7)
    com_p, com_o, _, _, _, _ = p.getLinkState(robotId, 8)
    rot_matrix = p.getMatrixFromQuaternion(com_o)
    rot_matrix = np.array(rot_matrix).reshape(3, 3)
    # Initial vectors
    init_camera_vector = (0, 0, 1)  # z-axis
    init_up_vector = (0, 1, 0)  # y-axis
    # Rotated vectors
    camera_vector = rot_matrix.dot(init_camera_vector)
    up_vector = rot_matrix.dot(init_up_vector)
    view_matrix = p.computeViewMatrix(com_p, com_p + 0.1 * camera_vector, up_vector)

    camera = Camera(
        fov=fov,
        near=nearplane,
        far=farplane,
        target_position=com_p,
        view_matrix=view_matrix,
        projection_matrix=projection_matrix,
    )

    return camera
