from typing import Callable, List

import numpy as np
from spot.utils.pcd_utils import transform_pcd

TABLE_HEIGHT = 0.65
BLOCK_SIZE = 0.05
Z_THRESH = 0.01
XY_THRESH = 0.015


def table_bussing_goal(cfg, pcd, pcd_seg):
    np.random.seed(cfg.seed)

    # pcd, pcd_seg = remove_outliers_from_full_pcd(cfg, point_cloud, seg, cfg.collision.inlier_ratio, cfg.collision.radius)

    extrinsics = np.load(cfg.extrinsics_file)
    T_cam_to_world = extrinsics["T"]

    pcd = transform_pcd(pcd, T_cam_to_world)

    object_names = cfg.real_world.object_names

    # Get the object names:
    object_list = [item.strip() for item in object_names.split(".") if item.strip()]
    num_objects = len(object_list)

    # Get the object ids:
    plates = []
    bowls = []
    cups = []
    for i, obj in enumerate(object_list):
        if "plate" in obj:
            plates.append(i)
        elif "bowl" in obj:
            bowls.append(i)
        elif "cup" in obj:
            cups.append(i)

    if len(plates) > 0:
        reference = plates[0]  # np.random.choice(plates)
        plates.remove(reference)
        align_thresh = cfg.heuristic.plate_align_thresh
        ref_obj = "plate"
    elif len(bowls) > 0:
        reference = bowls[0]  # np.random.choice(bowls)
        bowls.remove(reference)
        align_thresh = cfg.heuristic.bowl_align_thresh
        ref_obj = "bowl"
    else:
        raise ValueError(
            "No plates or bowls in the scene, Table bussing is not possible."
        )

    reference_pcd = pcd[pcd_seg == reference]
    reference_point = (
        np.max(reference_pcd[:, :2], axis=0) + np.min(reference_pcd[:, :2], axis=0)
    ) / 2
    stacking_align_thresh = cfg.heuristic.stacking_align_thresh

    # debug_ref_pcd = pcd - reference_mean
    # plot_pcd(debug_ref_pcd, pcd_seg, frame=True)

    # debug_ref_pcd = pcd - np.append(reference_point, 0.)
    # plot_pcd(debug_ref_pcd, pcd_seg, frame=True)

    # debug_ref_pcd = pcd - (np.append(reference_point, 0.) + np.array([align_thresh, 0., 0.]))
    # plot_pcd(debug_ref_pcd, pcd_seg, frame=True)

    # Heuristic is the number of objects that are not in place:
    h = num_objects - 1  # Because the reference object is already in place

    for plate in plates:
        thresh = stacking_align_thresh

        plate_pcd = pcd[pcd_seg == plate]
        plate_mean = (np.max(plate_pcd, axis=0) + np.min(plate_pcd, axis=0)) / 2

        if np.linalg.norm(plate_mean[:2] - reference_point) < thresh:
            h = h - 1

    for bowl in bowls:
        if ref_obj == "bowl":
            thresh = stacking_align_thresh  # If the reference object is a bowl, then the bowl should be stacked on top of the bowl
        else:
            thresh = align_thresh  # If the reference object is a plate, then the bowl should be aligned with the plate

        bowl_pcd = pcd[pcd_seg == bowl]
        # bowl_mean = np.mean(bowl_pcd, axis = 0)
        bowl_mean = (np.max(bowl_pcd, axis=0) + np.min(bowl_pcd, axis=0)) / 2

        if np.linalg.norm(bowl_mean[:2] - reference_point) < thresh:
            h = h - 1

    for cup in cups:
        thresh = align_thresh

        cup_pcd = pcd[pcd_seg == cup]
        # cup_mean = np.mean(cup_pcd, axis = 0)
        cup_mean = (np.max(cup_pcd, axis=0) + np.min(cup_pcd, axis=0)) / 2

        if np.linalg.norm(cup_mean[:2] - reference_point) < thresh:
            h = h - 1

    return h == 0


def table_bussing_heuristic(cfg, pcd, pcd_seg):
    np.random.seed(cfg.seed)

    # pcd, pcd_seg = remove_outliers_from_full_pcd(cfg, point_cloud, seg, cfg.collision.inlier_ratio, cfg.collision.radius)

    extrinsics = np.load(cfg.extrinsics_file)
    T_cam_to_world = extrinsics["T"]

    pcd = transform_pcd(pcd, T_cam_to_world)

    object_names = cfg.real_world.object_names

    # Get the object names:
    object_list = [item.strip() for item in object_names.split(".") if item.strip()]

    # Get the object ids:
    plates = []
    bowls = []
    cups = []
    for i, obj in enumerate(object_list):
        if "plate" in obj:
            plates.append(i)
        elif "bowl" in obj:
            bowls.append(i)
        elif "cup" in obj:
            cups.append(i)

    if len(plates) > 0:
        reference = plates[0]  # np.random.choice(plates)
        plates.remove(reference)
    elif len(bowls) > 0:
        reference = bowls[0]  # np.random.choice(bowls)
        bowls.remove(reference)
    else:
        raise ValueError(
            "No plates or bowls in the scene, Table bussing is not possible."
        )

    reference_pcd = pcd[pcd_seg == reference]
    # reference_mean = np.mean(reference_pcd, axis = 0)
    # Reference point is the middle of the reference object:
    reference_point = (
        np.max(reference_pcd[:, :2], axis=0) + np.min(reference_pcd[:, :2], axis=0)
    ) / 2

    # Heuristic is the number of objects that are not in place:
    h = 0.0  # Because the reference object is already in place

    for plate in plates:
        plate_pcd = pcd[pcd_seg == plate]
        plate_mean = np.mean(plate_pcd, axis=0)

        # h += np.linalg.norm(plate_mean[:2] - reference_mean[:2])
        h += np.linalg.norm(plate_mean[:2] - reference_point)

    for bowl in bowls:
        bowl_pcd = pcd[pcd_seg == bowl]
        bowl_mean = np.mean(bowl_pcd, axis=0)

        # debug_ref_pcd = pcd - bowl_mean
        # plot_pcd(debug_ref_pcd, pcd_seg, frame=True)

        # h += np.linalg.norm(bowl_mean[:2] - reference_mean[:2])
        h += np.linalg.norm(bowl_mean[:2] - reference_point)

    for cup in cups:
        cup_pcd = pcd[pcd_seg == cup]
        cup_mean = np.mean(cup_pcd, axis=0)

        # debug_ref_pcd = pcd - cup_mean
        # plot_pcd(debug_ref_pcd, pcd_seg, frame=True)

        # h += np.linalg.norm(cup_mean[:2] - reference_mean[:2])
        h += np.linalg.norm(cup_mean[:2] - reference_point)

    return h


def table_bussing_heuristic_quadratic_xyz(cfg, pcd, pcd_seg):
    np.random.seed(cfg.seed)

    # pcd, pcd_seg = remove_outliers_from_full_pcd(cfg, point_cloud, seg, cfg.collision.inlier_ratio, cfg.collision.radius)

    extrinsics = np.load(cfg.extrinsics_file)
    T_cam_to_world = extrinsics["T"]

    pcd = transform_pcd(pcd, T_cam_to_world)

    object_names = cfg.real_world.object_names

    # Get the object names:
    object_list = [item.strip() for item in object_names.split(".") if item.strip()]

    # Get the object ids:
    plates = []
    bowls = []
    cups = []
    for i, obj in enumerate(object_list):
        if "plate" in obj:
            plates.append(i)
        elif "bowl" in obj:
            bowls.append(i)
        elif "cup" in obj:
            cups.append(i)

    if len(plates) > 0:
        reference = plates[0]  # np.random.choice(plates)
        plates.remove(reference)
        align_thresh = cfg.heuristic.plate_align_thresh
        ref_obj = "plate"
    elif len(bowls) > 0:
        reference = bowls[0]  # np.random.choice(bowls)
        bowls.remove(reference)
        align_thresh = cfg.heuristic.bowl_align_thresh
        ref_obj = "bowl"
    else:
        raise ValueError(
            "No plates or bowls in the scene, Table bussing is not possible."
        )

    reference_pcd = pcd[pcd_seg == reference]
    reference_mean = np.mean(reference_pcd, axis=0)

    # Heuristic is the number of objects that are not in place:
    h = 0.0  # Because the reference object is already in place

    for plate in plates:
        thresh = cfg.heuristic.stacking_align_thresh

        plate_pcd = pcd[pcd_seg == plate]
        plate_mean = np.mean(plate_pcd, axis=0)

        h += ((1 / thresh) * np.linalg.norm(plate_mean - reference_mean)) ** 2

    for bowl in bowls:
        if ref_obj == "bowl":
            thresh = (
                cfg.heuristic.stacking_align_thresh
            )  # If the reference object is a bowl, then the bowl should be stacked on top of the bowl
        else:
            thresh = align_thresh  # If the reference object is a plate, then the bowl should be aligned with the plate

        bowl_pcd = pcd[pcd_seg == bowl]
        bowl_mean = np.mean(bowl_pcd, axis=0)

        h += ((1 / thresh) * np.linalg.norm(bowl_mean - reference_mean)) ** 2

    for cup in cups:
        thresh = align_thresh

        cup_pcd = pcd[pcd_seg == cup]
        cup_mean = np.mean(cup_pcd, axis=0)

        h += ((1 / thresh) * np.linalg.norm(cup_mean - reference_mean)) ** 2

    return h


def table_bussing_general(cfg, pcd, pcd_seg):
    np.random.seed(cfg.seed)

    # pcd, pcd_seg = remove_outliers_from_full_pcd(cfg, point_cloud, seg, cfg.collision.inlier_ratio, cfg.collision.radius)

    extrinsics = np.load(cfg.extrinsics_file)
    T_cam_to_world = extrinsics["T"]

    pcd = transform_pcd(pcd, T_cam_to_world)

    object_names = cfg.real_world.object_names

    # Get the object names:
    object_list = [item.strip() for item in object_names.split(".") if item.strip()]
    num_objects = len(object_list)

    # Get the object ids:
    plates = []
    bowls = []
    cups = []
    for i, obj in enumerate(object_list):
        if "plate" in obj:
            plates.append(i)
        elif "bowl" in obj:
            bowls.append(i)
        elif "cup" in obj:
            cups.append(i)

    if len(plates) > 0:
        reference = plates[0]  # np.random.choice(plates)
        plates.remove(reference)
        align_thresh = cfg.heuristic.plate_align_thresh
        ref_obj = "plate"
    elif len(bowls) > 0:
        reference = bowls[0]  # np.random.choice(bowls)
        bowls.remove(reference)
        align_thresh = cfg.heuristic.bowl_align_thresh
        ref_obj = "bowl"
    else:
        raise ValueError(
            "No plates or bowls in the scene, Table bussing is not possible."
        )

    reference_pcd = pcd[pcd_seg == reference]
    reference_mean = np.mean(reference_pcd, axis=0)
    stacking_align_thresh = cfg.heuristic.stacking_align_thresh

    # Heuristic is the number of objects that are not in place:
    h = num_objects - 1  # Because the reference object is already in place

    for plate in plates:
        thresh = stacking_align_thresh

        plate_pcd = pcd[pcd_seg == plate]
        plate_mean = np.mean(plate_pcd, axis=0)

        if np.linalg.norm(plate_mean[:2] - reference_mean[:2]) < thresh:
            h = h - 1

    for bowl in bowls:
        if ref_obj == "bowl":
            thresh = stacking_align_thresh  # If the reference object is a bowl, then the bowl should be stacked on top of the bowl
        else:
            thresh = align_thresh  # If the reference object is a plate, then the bowl should be aligned with the plate

        bowl_pcd = pcd[pcd_seg == bowl]
        bowl_mean = np.mean(bowl_pcd, axis=0)

        if np.linalg.norm(bowl_mean[:2] - reference_mean[:2]) < thresh:
            h = h - 1

    for cup in cups:
        thresh = align_thresh

        cup_pcd = pcd[pcd_seg == cup]
        cup_mean = np.mean(cup_pcd, axis=0)

        if np.linalg.norm(cup_mean[:2] - reference_mean[:2]) < thresh:
            h = h - 1

    return h


def table_bussing_two_plates_cup_and_bowl(cfg, pcd, pcd_seg):
    # TODO: Seg ID should be consistent across examples. Figure out a way to do this.

    # 0 -> cup
    # 1 -> plate
    # 2 -> plate
    # 3 -> bowl

    align_thresh_plate = 0.2
    plate_width = 0.25
    table_height = -0.05

    align_thresh = plate_width / 4

    # Initialize h as no. of objects:
    h = cfg.planner.num_objects

    extrinsics = np.load(cfg.extrinsics_file)
    T_cam_to_world = extrinsics["T"]

    # TODO: I will just check if the mean of the points are at the correct place for now. Need to handle rotations later too.
    # TODO: Assign to each object a mask whether that object is in place. Then visualize this point cloud to explain the heuristic

    pcd_world = transform_pcd(pcd, T_cam_to_world)

    plate_pcd_1 = pcd_world[pcd_seg == 1]
    plate_pcd_2 = pcd_world[pcd_seg == 2]
    bowl_pcd = pcd_world[pcd_seg == 3]
    cup_pcd = pcd_world[pcd_seg == 0]

    plate_1_mean = np.mean(plate_pcd_1, axis=0)
    plate_2_mean = np.mean(plate_pcd_2, axis=0)
    bowl_mean = np.mean(bowl_pcd, axis=0)
    cup_mean = np.mean(cup_pcd, axis=0)

    all_objects_above_table = False
    plates_stacked = False
    plates_aligned = False
    bowl_aligned = False
    cup_aligned = False

    # Heuristic 3: All objects on table
    if (
        (plate_1_mean[2] > table_height)
        and (plate_2_mean[2] > table_height)
        and (bowl_mean[2] > table_height)
        and (cup_mean[2] > table_height)
    ):
        h -= 1
        all_objects_above_table = True

    # Heuristic 2:
    if (
        np.linalg.norm(plate_1_mean[:2] - plate_2_mean[:2]) < align_thresh_plate
    ) and all_objects_above_table:
        plates_aligned = True
    if (plate_2_mean[2] > plate_1_mean[2]) and plates_aligned:
        h -= 1
        top_plate_mean = plate_2_mean
        plates_stacked = True
    elif (plate_1_mean[2] > plate_2_mean[2]) and plates_aligned:
        h -= 1
        top_plate_mean = plate_1_mean
        plates_stacked = True

    if plates_stacked and plates_aligned:
        # TODO: For this, a better method would be to fit a circle to the plate's points on the (x,y) plane and then check if some % of the
        # cup's and bowl's points lie inside the circle. Then check if majority of the points are above the plate.

        # Heuristic 1: Bowl is on plate
        if np.linalg.norm(bowl_mean[:2] - top_plate_mean[:2]) < align_thresh:
            h -= 1
            bowl_aligned = True

        # Heuristic 0: Cup is on plate
        if np.linalg.norm(cup_mean[:2] - top_plate_mean[:2]) < align_thresh:
            h -= 1
            cup_aligned = True

    return h


def _estimate_block_pose(pcd: np.ndarray) -> np.ndarray:
    """Estimate x, y, z pose of block point cloud."""
    x_min = pcd[:, 0].min()
    x_max = pcd[:, 0].max()
    y_min = pcd[:, 1].min()
    y_max = pcd[:, 1].max()
    z_min = pcd[:, 2].min()
    z_max = pcd[:, 2].max()

    return np.array(
        [
            (x_min + x_max) / 2,
            (y_min + y_max) / 2,
            (z_min + z_max) / 2,
        ]
    )


def _get_block_poses(pcd: np.ndarray, pcd_seg: np.ndarray):
    """
    Args:
    - pcd: (n_points, 3)
    - pcd_seg: (n_points, )

    Segmentation values:
    - 2 -> red block
    - 3 -> green block
    - 4 -> blue block
    """
    R_pcd = pcd[pcd_seg == 2]
    G_pcd = pcd[pcd_seg == 3]
    B_pcd = pcd[pcd_seg == 4]

    R = _estimate_block_pose(R_pcd)
    G = _estimate_block_pose(G_pcd)
    B = _estimate_block_pose(B_pcd)
    return R, G, B


class Heuristic:
    def __init__(self, fn, name):
        self.fn: Callable = fn
        self.name: str = name

    def __call__(self, cfg, pcd: np.ndarray, pcd_seg: np.ndarray):
        return self.fn(cfg, pcd, pcd_seg)


def block_stacking_RGB(cfg, pcd: np.ndarray, pcd_seg: np.ndarray):
    h = 3
    R, G, B = _get_block_poses(pcd, pcd_seg)

    # Goal is RGB from top to bottom:
    if np.abs(B[2] - TABLE_HEIGHT) < Z_THRESH:
        # This means blue cube is on the table
        h -= 1

        if (
            np.abs(G[2] - B[2] - BLOCK_SIZE) < Z_THRESH
            and np.linalg.norm((G[:2] - B[:2])) < XY_THRESH
        ):
            # Also, green is on blue
            h -= 1

            if (
                np.abs(R[2] - G[2] - BLOCK_SIZE) < Z_THRESH
                and np.linalg.norm((R[:2] - G[:2])) < XY_THRESH
            ):
                # Additionally, red is on top of green
                h -= 1

    return h


def block_stacking_RBG(cfg, pcd: np.ndarray, pcd_seg: np.ndarray):
    h = 3
    R, G, B = _get_block_poses(pcd, pcd_seg)

    # Goal is RBG from top to bottom:
    if np.abs(G[2] - TABLE_HEIGHT) < Z_THRESH:
        # This means green cube is on the table
        h -= 1

        if (
            np.abs(B[2] - G[2] - BLOCK_SIZE) < Z_THRESH
            and np.linalg.norm((B[:2] - G[:2])) < XY_THRESH
        ):
            # Also, blue is on green
            h -= 1

            if (
                np.abs(R[2] - B[2] - BLOCK_SIZE) < Z_THRESH
                and np.linalg.norm((R[:2] - B[:2])) < XY_THRESH
            ):
                # Additionally, red is on top of blue
                h -= 1

    return h


def block_stacking_GRB(cfg, pcd: np.ndarray, pcd_seg: np.ndarray):
    h = 3
    R, G, B = _get_block_poses(pcd, pcd_seg)

    # Goal is GRB from top to bottom:
    if np.abs(B[2] - TABLE_HEIGHT) < Z_THRESH:
        # This means blue cube is on the table
        h -= 1

        if (
            np.abs(R[2] - B[2] - BLOCK_SIZE) < Z_THRESH
            and np.linalg.norm((R[:2] - B[:2])) < XY_THRESH
        ):
            # Also, red is on blue
            h -= 1

            if (
                np.abs(G[2] - R[2] - BLOCK_SIZE) < Z_THRESH
                and np.linalg.norm((G[:2] - R[:2])) < XY_THRESH
            ):
                # Additionally, green is on top of red
                h -= 1

    return h


def block_stacking_GBR(cfg, pcd: np.ndarray, pcd_seg: np.ndarray):
    h = 3
    R, G, B = _get_block_poses(pcd, pcd_seg)

    # Goal is GBR from top to bottom:
    if np.abs(R[2] - TABLE_HEIGHT) < Z_THRESH:
        # This means red cube is on the table
        h -= 1

        if (
            np.abs(B[2] - R[2] - BLOCK_SIZE) < Z_THRESH
            and np.linalg.norm((B[:2] - R[:2])) < XY_THRESH
        ):
            # Also, blue is on red
            h -= 1

            if (
                np.abs(G[2] - B[2] - BLOCK_SIZE) < Z_THRESH
                and np.linalg.norm((G[:2] - B[:2])) < XY_THRESH
            ):
                # Additionally, green is on top of blue
                h -= 1

    return h


def block_stacking_BRG(cfg, pcd: np.ndarray, pcd_seg: np.ndarray):
    h = 3
    R, G, B = _get_block_poses(pcd, pcd_seg)

    # Goal is BRG from top to bottom:
    if np.abs(G[2] - TABLE_HEIGHT) < Z_THRESH:
        # This means green cube is on the table
        h -= 1

        if (
            np.abs(R[2] - G[2] - BLOCK_SIZE) < Z_THRESH
            and np.linalg.norm((R[:2] - G[:2])) < XY_THRESH
        ):
            # Also, red is on green
            h -= 1

            if (
                np.abs(B[2] - R[2] - BLOCK_SIZE) < Z_THRESH
                and np.linalg.norm((B[:2] - R[:2])) < XY_THRESH
            ):
                # Additionally, blue is on top of red
                h -= 1

    return h


def block_stacking_BGR(cfg, pcd: np.ndarray, pcd_seg: np.ndarray):
    h = 3
    R, G, B = _get_block_poses(pcd, pcd_seg)

    # Goal is BGR from top to bottom:
    if np.abs(R[2] - TABLE_HEIGHT) < Z_THRESH:
        # This means red cube is on the table
        h -= 1

        if (
            np.abs(G[2] - R[2] - BLOCK_SIZE) < Z_THRESH
            and np.linalg.norm((G[:2] - R[:2])) < XY_THRESH
        ):
            # Also, green is on red
            h -= 1

            if (
                np.abs(B[2] - G[2] - BLOCK_SIZE) < Z_THRESH
                and np.linalg.norm((B[:2] - G[:2])) < XY_THRESH
            ):
                # Additionally, blue is on top of green
                h -= 1

    return h


BLOCKS_STACK_HEURISTICS: List[Heuristic] = [
    Heuristic(block_stacking_RGB, "RGB"),
    Heuristic(block_stacking_RBG, "RBG"),
    Heuristic(block_stacking_GRB, "GRB"),
    Heuristic(block_stacking_GBR, "GBR"),
    Heuristic(block_stacking_BRG, "BRG"),
    Heuristic(block_stacking_BGR, "BGR"),
]


def table_bussing_two_plates_cup_and_bowl_per_object(cfg, pcd, pcd_seg):
    # TODO: Seg ID should be consistent across examples. Figure out a way to do this.

    # 0 -> cup
    # 1 -> plate
    # 2 -> plate
    # 3 -> bowl

    align_thresh_plate = 0.15
    plate_width = 0.15
    table_height = -0.05

    align_thresh = plate_width / 4

    # Initialize h as no. of objects:
    h = cfg.planner.num_objects

    extrinsics = np.load(cfg.extrinsics_file)
    T_cam_to_world = extrinsics["T"]

    # TODO: I will just check if the mean of the points are at the correct place for now. Need to handle rotations later too.
    # TODO: Assign to each object a mask whether that object is in place. Then visualize this point cloud to explain the heuristic

    pcd_world = transform_pcd(pcd, T_cam_to_world)

    plate_pcd_1 = pcd_world[pcd_seg == 1]
    plate_pcd_2 = pcd_world[pcd_seg == 2]
    bowl_pcd = pcd_world[pcd_seg == 3]
    cup_pcd = pcd_world[pcd_seg == 0]

    plate_1_mean = np.mean(plate_pcd_1, axis=0)
    plate_2_mean = np.mean(plate_pcd_2, axis=0)
    bowl_mean = np.mean(bowl_pcd, axis=0)
    cup_mean = np.mean(cup_pcd, axis=0)

    one_plate_on_table = False
    plates_stacked = False
    plates_aligned = False
    bowl_aligned = False
    cup_aligned = False

    # if (plate_1_mean[2] > table_height) and (plate_2_mean[2] > table_height) and (bowl_mean[2] > table_height) and (cup_mean[2] > table_height):
    #     h -= 1
    #     all_objects_above_table = True

    # Heuristic 3: Atleast one plate on table
    if (plate_1_mean[2] > table_height) or (plate_2_mean[2] > table_height):
        h -= 1
        one_plate_on_table = True

    # Heuristic 2: Plates are stacked
    if np.linalg.norm(plate_1_mean[:2] - plate_2_mean[:2]) < align_thresh_plate:
        if plate_2_mean[2] > plate_1_mean[2]:
            h -= 1
            top_plate_mean = plate_2_mean
            plates_stacked = True
        elif plate_1_mean[2] > plate_2_mean[2]:
            h -= 1
            top_plate_mean = plate_1_mean
            plates_stacked = True

    # TODO: For this, a better method would be to fit a circle to the plate's points on the (x,y) plane and then check if some % of the
    # cup's and bowl's points lie inside the circle. Then check if majority of the points are above the plate.

    # Heuristic 1: Bowl is on a plate
    if (np.linalg.norm(bowl_mean[:2] - plate_1_mean[:2]) < align_thresh) or (
        np.linalg.norm(bowl_mean[:2] - plate_2_mean[:2]) < align_thresh
    ):
        h -= 1
        bowl_aligned = True

    # Heuristic 0: Cup is on plate
    if (np.linalg.norm(cup_mean[:2] - plate_1_mean[:2]) < align_thresh) or (
        np.linalg.norm(cup_mean[:2] - plate_2_mean[:2]) < align_thresh
    ):
        h -= 1
        cup_aligned = True

    return h
