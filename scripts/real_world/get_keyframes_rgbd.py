import argparse
import os
import sys
import time

import cv2
from omegaconf import OmegaConf
from pyk4a import PyK4A

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(current_dir)

import os

# from marker_detection import get_kinect_ir_frame, detect_aruco_markers, estimate_transformation, get_kinect_rgb_frame
import numpy as np


def get_kinect_rgbd_frame(device, visualize=False):
    """
    Capture an IR frame from the Kinect camera.
    """
    # Capture an IR frame
    rgb_frame = None
    capture = None
    for i in range(20):
        try:
            device.get_capture()
            capture = device.get_capture()
            if capture is not None:
                ir_frame = capture.ir

                # depth_frame = capture.depth
                # ---
                depth_frame = capture.transformed_depth
                # ---

                # cv2.imshow('IR', ir_frame)
                rgb_frame = capture.color
                gray_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2GRAY)
                gray_frame = np.clip(gray_frame, 0, 5e3) / 5e3  # Clip and normalize
                # cv2.imshow('color', rgb_frame)

                ir_frame_norm = np.clip(ir_frame, 0, 5e3) / 5e3  # Clip and normalize
                pcd_frame = capture.transformed_depth_point_cloud
                # print(pcd_frame.shape, ir_frame.shape)
                # print("successful capture")
                return ir_frame, rgb_frame, ir_frame_norm, pcd_frame, depth_frame
        except:
            time.sleep(0.1)
            # print("Failed to capture IR frame.")
    else:
        # print("Failed to capture IR frame after 20 attempts.")
        return None


def get_current_rgbd(fname, keyframes_folder):
    # print(i)
    time.sleep(0.1)
    ir_frame, rgb_frame, ir_frame_norm, pcd_frame, depth_frame = get_kinect_rgbd_frame(
        k4a
    )

    pts3d = pcd_frame.astype(np.float32) / 1e3  # Convert to meters
    depth_frame = depth_frame.astype(np.float32) / 1e3  # Convert to meters
    depth_frame[depth_frame > 0.95] = 0.0
    rgb = rgb_frame[:, :, :3]

    norm_depth = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX)
    inverted_depth = 255 - norm_depth
    depth_image_8bit = inverted_depth.astype(np.uint8)

    # cv2.imshow("Depth Image", depth_image_8bit)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # plot_pcd(pts3d, rgb)

    pts3d_path = os.path.join(keyframes_folder, fname + ".npy")
    rgb_path = os.path.join(keyframes_folder, fname + ".jpg")
    depth_path = os.path.join(keyframes_folder, fname + "_depth" + ".jpg")
    # np.save(depth_path, depth_frame)
    cv2.imwrite(depth_path, depth_image_8bit)
    np.save(pts3d_path, pts3d)
    cv2.imwrite(rgb_path, rgb)

    return pts3d, rgb


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Evaluate the planning algorithm")
    parser.add_argument(
        "--config", type=str, default="plan.yaml", help="config for the planner"
    )
    args = parser.parse_args()

    # with hydra.initialize(version_base=None, config_path=os.path.dirname(args.config)):
    #     cfg = hydra.compose(args.config)
    cfg = OmegaConf.load(args.config)

    k4a = PyK4A(device_id=cfg.real_world.cam_id)
    k4a.start()

    # Prepare folder to save keyframes in:
    keyframes_folder = os.path.join(cfg.plan_folder, "keyframes")
    os.makedirs(keyframes_folder, exist_ok=True)

    # folder_path = "MDE/keyframes/"
    # _ = input("Overwriting keyframes folder. Press Enter to continue.")
    file_names = os.listdir(keyframes_folder)
    numbers = [int(file_names.split(".")[0]) for file_names in file_names]
    largest_idx = max(numbers) if numbers else 0
    largest_idx += 1
    get_current_rgbd(str(largest_idx), keyframes_folder)
