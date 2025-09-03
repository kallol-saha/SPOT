from queue import PriorityQueue
from typing import Callable, Optional

import numpy as np
from spot.collision import CollisionChecker
from spot.mde import MDE
from spot.object_suggester import ObjectSuggester
from spot.suggester import Suggester
from spot.utils.pcd_utils import (
    downsample_pcd,
    remove_outliers_from_full_pcd,
    transform_object_pcd,
    transform_pcd,
)


class Node:
    """
    A node in a search tree.

    We assume that each node can have multiple children, but only one parent.
    """

    def __init__(
        self,
        cfg,
        h,
        id,
        moved_object=None,
        T=None,
        parent=None,
        deviation=0.0,
        is_pruned=False,
        collision=0.0,
        probability=1.0,
        is_goal=False,
        obj_probs=None,
        distribution=None,
    ):
        self.num_objects = cfg.planner.num_objects
        self.obj_ids = [int(i) for i in cfg.planner.object_ids.split(",")]
        self.id = id
        self.moved_object = moved_object
        self.parent = parent
        self.children = []

        self.obj_probs = obj_probs
        self.distribution = distribution

        # Flags that define the role of this node in the graph:
        self.is_pruned = is_pruned
        self.in_plan = False
        self.expanded = False
        self.is_goal = is_goal

        # Store predicted deviation of the transition to this node
        self.deviation = deviation
        self.collision = collision

        if parent is None:
            self.g = 0.0
            self.coll_cost = 0.0
            self.path_length = 0
            self.probability = 1.0
            self.T = np.eye(4)
            self.transforms = np.stack(
                [np.eye(4) for _ in range(self.num_objects)]
            )  # FIXME: Make this absolute transforms with respect to original point cloud

        else:
            # Assign the child to its parent
            self.parent.children.append(self)
            self.path_length = self.parent.path_length + 1
            self.probability = probability

            # Define the transforms
            self.T = T
            self.transforms = self.parent.transforms.copy()
            idx = self.obj_ids.index(moved_object)
            self.transforms[idx] = self.T @ self.transforms[idx]

            # Update g
            # self.coll_cost = (
            #     (self.parent.coll_cost * (self.path_length - 1)) + collision
            # ) / self.path_length  # Moving average of collision cost
            self.g = (
                self.parent.g
                + cfg.planner.action_cost
                + (cfg.collision.weight * self.collision)
                + (cfg.mde.weight * self.deviation)
                + (cfg.suggester.probability_weight * (1 - self.probability))
            )
            # self.g = self.parent.g + collision + np.linalg.norm(self.T[:3, -1])     # Movement cost is the edge cost

        # Update heuristic and f
        self.h = h
        self.f = self.g + self.h

    def __eq__(self, other):
        return self.id == other.id

    def __lt__(self, other):
        return self.f < other.f

    def get_pcd(self, initial_pcd, initial_pcd_seg):
        pcd = initial_pcd.copy()

        for i, obj_id in enumerate(self.obj_ids):
            pcd[initial_pcd_seg == obj_id] = transform_pcd(
                pcd[initial_pcd_seg == obj_id], self.transforms[i]
            )

        return pcd


class RandomRollout:
    def __init__(
        self,
        cfg,
        suggester: Suggester,
        mde: MDE,
        object_suggester: ObjectSuggester = None,
    ):
        self.cfg = cfg

        self.suggester: Suggester = suggester
        self.mde: MDE = mde
        self.object_suggester: ObjectSuggester = object_suggester

        if cfg.collision is not None:
            self.collision_checker = CollisionChecker(cfg.collision)

        self.action_threshold = cfg.planner.filter_small_actions

        self.num_objects = cfg.planner.num_objects
        self.obj_ids = [int(i) for i in cfg.planner.object_ids.split(",")]

        self.rng = np.random.default_rng(cfg.seed)

    def plan(
        self, initial_pcd, initial_pcd_seg, heuristic, goal: Optional[Callable] = None
    ):
        if goal is None:
            # By default, goal function returns True iff the heuristic equals 0
            def goal(cfg, pcd, pcd_seg):
                return heuristic(cfg, pcd, pcd_seg) == 0

        # Initialize open list, where nodes to be expanded are listed
        open_list = PriorityQueue()
        num_nodes = 0
        expanded_nodes = 0
        pruned_nodes = 0
        self.num_goals = 0

        initial_pcd_vis, initial_pcd_seg_vis = remove_outliers_from_full_pcd(
            self.cfg,
            initial_pcd,
            initial_pcd_seg,
            self.cfg.collision.inlier_ratio,
            self.cfg.collision.radius,
        )

        if self.cfg.planner.downsample:
            # Remove non-object points
            initial_pcd_obj = initial_pcd[np.isin(initial_pcd_seg, self.obj_ids)]
            initial_pcd_seg_obj = initial_pcd_seg[
                np.isin(initial_pcd_seg, self.obj_ids)
            ]

            initial_pcd_mde, initial_pcd_seg_mde = downsample_pcd(
                initial_pcd_obj, initial_pcd_seg_obj, self.cfg.num_points
            )
        else:
            initial_pcd_mde, initial_pcd_seg_mde = initial_pcd, initial_pcd_seg

        # Define the start node and add it to the queue
        h = heuristic(self.cfg, initial_pcd_vis, initial_pcd_seg_vis)
        start_node = Node(self.cfg, h, id=num_nodes)
        open_list.put((start_node.f, start_node))
        num_nodes += 1

        # Loop until there is nothing to loop over
        while (
            not open_list.empty()
            and expanded_nodes < self.cfg.planner.max_expanded_nodes
            and self.num_goals < self.cfg.planner.max_goals
        ):
            # Get the current node and add to closed list:
            _, current_node = open_list.get()  # Returns heuristic and node object
            current_node.expanded = True
            # closed_list.append(current_node)

            current_pcd = current_node.get_pcd(initial_pcd, initial_pcd_seg)
            current_pcd_vis = current_node.get_pcd(initial_pcd_vis, initial_pcd_seg_vis)
            current_pcd_mde = current_node.get_pcd(initial_pcd_mde, initial_pcd_seg_mde)

            if goal(
                self.cfg, current_pcd_vis, initial_pcd_seg_vis
            ):  # If current node is the goal
                # Create the plan by backtracking

                plan = {}
                transforms = []
                object_order = []
                node = current_node
                node.is_goal = True
                self.num_goals += 1

                while node.parent is not None:
                    # states.append(current.state)
                    node.in_plan = True
                    transforms.append(node.T)
                    object_order.append(node.moved_object)
                    node = node.parent

                # Return the reversed plans
                plan["transforms"] = np.stack(transforms, axis=0)[::-1]
                plan["object_order"] = np.array(object_order)[::-1]

                # Here the returned node is the start node, which should have all
                # info to create the graph
                # return (
                #     plan,
                #     start_node,
                #     pruned_nodes,
                # )

            expanded_nodes += 1
            # Print with replacement:
            print(
                f"\rExpanded {expanded_nodes} nodes, found {self.num_goals} goals",
                end="",
                flush=True,
            )

            obj_probs, p = self.object_suggester.predict(
                current_pcd_mde, initial_pcd_seg_mde, self.obj_ids
            )
            # Choose a random object:
            obj = self.rng.choice(self.obj_ids)
            current_node.obj_probs = p

            # Moving the same object twice in a row does not make sense
            if current_node.parent is not None and current_node.moved_object == obj:
                open_list.put((start_node.f, start_node))
                continue

            if current_node.path_length >= self.cfg.planner.max_path_length:
                open_list.put((start_node.f, start_node))
                continue

            proposed_transforms, probs, dists = self.suggester.suggest(
                current_pcd, initial_pcd_seg, obj
            )  # Returns a list of transforms

            i = self.rng.choice(len(proposed_transforms))  # Choose a random transform
            T = proposed_transforms[i]
            prob = probs[i]
            distribution = dists[i]

            child_node_pcd_vis = transform_object_pcd(
                current_pcd_vis, initial_pcd_seg_vis, T, obj
            )
            h = heuristic(self.cfg, child_node_pcd_vis, initial_pcd_seg_vis)
            child_node = Node(
                self.cfg,
                h=h,
                id=num_nodes,
                moved_object=obj,
                T=T,
                parent=current_node,
                collision=0.0,
                probability=obj_probs[obj] * prob,
                distribution=distribution,
            )
            num_nodes += 1

            open_list.put((child_node.f, child_node))

        return (
            None,
            start_node,
            pruned_nodes,
        )  # The start_node contains all the info to create the graph
