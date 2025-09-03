import os

import graphviz as gv
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

from spot.utils.pcd_utils import COLORS, remove_outliers_from_full_pcd


def get_colors(pcd_seg, seg_colors):
    # id_to_color = {1: "gray", 2: "red", 3: "green", 4: "blue"}  # NOTE from Kallol: I moved this to config as seg_colors, the function still does the same thing
    colors = np.zeros((pcd_seg.shape[0], 3))
    for seg_id in np.unique(pcd_seg):
        colors[pcd_seg == seg_id] = COLORS[seg_colors[int(seg_id)]]

    return colors


def get_o3d_pcd(pcd, pcd_seg, seg_colors: dict = None):
    pcd_vis = o3d.geometry.PointCloud()
    pcd_vis.points = o3d.utility.Vector3dVector(pcd)

    if seg_colors is not None:
        colors = get_colors(pcd_seg, seg_colors)
    else:
        seg_ids = np.unique(pcd_seg)
        n = len(seg_ids)
        cmap = plt.get_cmap("tab10")
        id_to_color = {uid: cmap(i / n)[:3] for i, uid in enumerate(seg_ids)}

        colors = np.array([id_to_color[seg_id] for seg_id in pcd_seg])

    pcd_vis.colors = o3d.utility.Vector3dVector(colors)
    return pcd_vis


def visualize_node(node, initial_pcd, initial_pcd_seg, cfg):
    pcd = node.get_pcd(initial_pcd, initial_pcd_seg)
    pcd_vis = get_o3d_pcd(pcd, initial_pcd_seg, seg_colors=cfg.visual.seg_colors)

    # Save node info:
    # if node.id != 0:
    #     node_info_save_folder = os.path.join(cfg.plan_folder, "node_info")
    #     os.makedirs(node_info_save_folder, exist_ok=True)
    #     node_path = os.path.join(node_info_save_folder, f"node_{node.id}.npz")
    #     np.savez(
    #         node_path,
    #         pcd=pcd,
    #         pcd_seg=initial_pcd_seg,
    #         moved_object=node.moved_object,
    #         T=node.T,
    #         parent=node.parent.id,
    #         is_goal=node.is_goal,
    #         deviation=node.deviation,
    #         collision=node.collision,
    #         probability=node.probability,
    #         distribution=node.distribution,
    #         obj_probs=node.obj_probs,
    #     )
    # np.save(pcd_path, pcd)

    # Save plan information:

    print(f"\rVisualized {node.id} nodes", end="", flush=True)

    # # Offscreen rendering setup
    # width, height = 512, 512  # Image dimensions
    # renderer = o3d.visualization.rendering.OffscreenRenderer(width, height)

    # # Add the point cloud to the scene
    # material = o3d.visualization.rendering.MaterialRecord()  # Default material
    # renderer.scene.add_geometry("point_cloud", pcd_vis, material)

    # # Render and save the image
    # image = renderer.render_to_image()

    # Step 2: Set up Open3D visualization and render
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False, width=512, height=512)  # Set window size
    vis.add_geometry(pcd_vis)

    # Set point size
    render_option = vis.get_render_option()
    render_option.point_size = cfg.visual.point_size

    ctr = vis.get_view_control()
    camera_params = ctr.convert_to_pinhole_camera_parameters()
    extrinsic = np.eye(4)
    extrinsic[2, 3] = cfg.visual.camera_zoom_out  # Zoom out to capture everything
    camera_params.extrinsic = extrinsic
    ctr.convert_from_pinhole_camera_parameters(camera_params, allow_arbitrary=True)

    if cfg.visual.camera_params is not None:
        ctr.set_front(cfg.visual.camera_params.front)
        ctr.set_lookat(cfg.visual.camera_params.lookat)
        ctr.set_up(cfg.visual.camera_params.up)
        ctr.set_zoom(cfg.visual.camera_params.zoom)

    # Render and capture the view
    vis.poll_events()
    vis.update_renderer()
    image = vis.capture_screen_float_buffer(do_render=True)

    # Step 3: Convert image to Open3D Image and save
    image_np = (np.asarray(image) * 255).astype(
        np.uint8
    )  # Convert float buffer to uint8
    image_o3d = o3d.geometry.Image(image_np)  # Convert NumPy array to Open3D Image
    # output_image_path = "random_point_cloud_image.png"
    # o3d.io.write_image(output_image_path, image_o3d)

    # Clean up
    vis.destroy_window()

    return image_o3d


def add_nodes_in_plan(node, graph, initial_pcd, initial_pcd_seg, node_images_dir, cfg):
    # Get rendered image of node:
    image_file_path = os.path.join(node_images_dir, str(node.id) + ".png")

    # Add node to the graph
    graph.node(
        str(node.id),
        label=f"g={np.round(node.g, 2)}, h={np.round(node.h, 2)} \n f={np.round(node.f, 2)}, id={node.id} \n coll={np.round(node.collision, 2)}, dev={np.round(node.deviation, 2)} \n prob={np.round(node.probability, 2)}",  # \n moved {node.moved_object}",
        image=os.path.join(os.getcwd(), image_file_path),
        penwidth="2",
        imagescale="true",
        width="1.5",
        height="1.5",
        shape="square",
        labelloc="b",
    )

    # Add edge to the graph:
    if node.parent is not None:
        graph.edge(
            str(node.parent.id),
            str(node.id),
            color="black",
            label="",
            penwidth="10",
        )

    # Loop through the children and recurse the function:
    for child in node.children:
        if child.in_plan:
            graph = add_nodes_in_plan(
                child, graph, initial_pcd, initial_pcd_seg, node_images_dir, cfg
            )

    return graph


def add_nodes_expanded(node, graph, initial_pcd, initial_pcd_seg, node_images_dir, cfg):
    # Get rendered image of node:
    image_file_path = os.path.join(node_images_dir, str(node.id) + ".png")

    # Add node to the graph
    graph.node(
        str(node.id),
        label=f"g={np.round(node.g, 2)}, h={np.round(node.h, 2)} \n f={np.round(node.f, 2)}, id={node.id} \n coll={np.round(node.collision, 2)}, dev={np.round(node.deviation, 2)} \n prob={np.round(node.probability, 2)}",  # \n moved {node.moved_object}",
        image=os.path.join(os.getcwd(), image_file_path),
        penwidth="2",
        imagescale="true",
        width="1.5",
        height="1.5",
        shape="square",
        labelloc="b",
    )

    # Add edge to the graph:
    if node.parent is not None:
        if node.in_plan:
            color = "green"
        else:
            color = "blue"

        graph.edge(
            str(node.parent.id),
            str(node.id),
            color=color,
            label="",
            penwidth="10",
        )

    # Loop through the children and recurse the function:
    for child in node.children:
        if child.expanded:
            graph = add_nodes_expanded(
                child, graph, initial_pcd, initial_pcd_seg, node_images_dir, cfg
            )

    return graph


def add_nodes_not_pruned(
    node, graph, initial_pcd, initial_pcd_seg, node_images_dir, cfg
):
    # Get rendered image of node:
    image_file_path = os.path.join(node_images_dir, str(node.id) + ".png")

    # Add node to the graph
    graph.node(
        str(node.id),
        label=f"g={np.round(node.g, 2)}, h={np.round(node.h, 2)} \n f={np.round(node.f, 2)}, id={node.id} \n coll={np.round(node.collision, 2)}, dev={np.round(node.deviation, 2)} \n prob={np.round(node.probability, 2)}",  # \n moved {node.moved_object}",
        image=os.path.join(os.getcwd(), image_file_path),
        penwidth="2",
        imagescale="true",
        width="1.5",
        height="1.5",
        shape="square",
        labelloc="b",
    )

    # Add edge to the graph:
    if node.parent is not None:
        # Assign color to the edge:
        if node.expanded and node.in_plan:
            color = "green"
        elif node.expanded:
            color = "blue"
        else:
            color = "black"

        graph.edge(
            str(node.parent.id),
            str(node.id),
            color=color,
            label="",
            penwidth="10",
        )

    # Loop through the children and recurse the function:
    for child in node.children:
        if not child.is_pruned:
            graph = add_nodes_not_pruned(
                child, graph, initial_pcd, initial_pcd_seg, node_images_dir, cfg
            )

    return graph


def add_nodes_recursively(
    node, graph, initial_pcd, initial_pcd_seg, node_images_dir, cfg
):
    # Get rendered image of node:
    node_image = visualize_node(node, initial_pcd, initial_pcd_seg, cfg)
    image_file_path = os.path.join(node_images_dir, str(node.id) + ".png")
    # Save the image:
    # output_image_path = "random_point_cloud_image.png"
    o3d.io.write_image(image_file_path, node_image)
    # o3d.io.write_image(image_file_path, node_image)

    # Add node to the graph
    graph.node(
        str(node.id),
        label=f"g={np.round(node.g, 2)}, h={np.round(node.h, 2)} \n f={np.round(node.f, 2)}, id={node.id} \n coll={np.round(node.collision, 2)}, dev={np.round(node.deviation, 2)} \n prob={np.round(node.probability, 2)}",
        image=os.path.join(os.getcwd(), image_file_path),
        penwidth="2",
        imagescale="true",
        width="1.5",
        height="1.5",
        shape="square",
        labelloc="b",
    )

    # Add edge to the graph:
    if node.parent is not None:
        # Assign color to the edge:
        if node.expanded and node.in_plan:
            color = "green"
        elif node.expanded:
            color = "blue"
        elif not node.is_pruned:
            color = "black"
        else:
            color = "grey"

        graph.edge(
            str(node.parent.id),
            str(node.id),
            color=color,
            # label=f"deviation: {node.deviation:9.5f}",
            penwidth="10",
        )

    # Loop through the children and recurse the function:
    for child in node.children:
        if child.expanded:
            graph = add_nodes_recursively(
                child, graph, initial_pcd, initial_pcd_seg, node_images_dir, cfg
            )

    return graph


def create_graph(
    cfg,
    start_node,
    initial_pcd,
    initial_pcd_seg,
    output_dir,
    remove_outliers: bool = True,
):
    # output_dir = "experiments/" + cfg.experiment_name + "/"
    if remove_outliers:
        initial_pcd, initial_pcd_seg = remove_outliers_from_full_pcd(
            cfg,
            initial_pcd,
            initial_pcd_seg,
            inlier_ratio=cfg.collision.inlier_ratio,
            radius=cfg.collision.radius,
        )

    node_images_dir = os.path.join(output_dir, "node_images")
    os.makedirs(node_images_dir, exist_ok=True)

    # graph = gv.Digraph(format="png", strict=True)  # Initialize a directed graph
    # if cfg.visual.graphs.full_graph:

    graph = (
        gv.Digraph()
    )  # NOTE from Kallol: personally like the default pdf format, it loads faster
    graph = add_nodes_recursively(
        start_node, graph, initial_pcd, initial_pcd_seg, node_images_dir, cfg
    )
    graph.render(
        os.path.join(output_dir, "full_graph"), view=False
    )  # This should save both the graph, and the pdf visualization

    if cfg.visual.graphs.plan_graph:
        # Saving the plan as a separate graph
        plan_graph = gv.Digraph()
        plan_graph = add_nodes_in_plan(
            start_node, plan_graph, initial_pcd, initial_pcd_seg, node_images_dir, cfg
        )
        plan_graph.render(os.path.join(output_dir, "plan_graph"), view=False)

    if cfg.visual.graphs.expanded_graph:
        # Graph of nodes expanded
        expanded_graph = gv.Digraph()
        expanded_graph = add_nodes_expanded(
            start_node,
            expanded_graph,
            initial_pcd,
            initial_pcd_seg,
            node_images_dir,
            cfg,
        )
        expanded_graph.render(os.path.join(output_dir, "expanded_graph"), view=False)

    if cfg.visual.graphs.pruned_graph:
        # Graph of nodes not pruned
        pruned_graph = gv.Digraph()
        pruned_graph = add_nodes_not_pruned(
            start_node, pruned_graph, initial_pcd, initial_pcd_seg, node_images_dir, cfg
        )
        pruned_graph.render(os.path.join(output_dir, "pruned_graph"), view=False)

    # TODO: Might also want to save the graph as a transitional video in addition to the pdf.
