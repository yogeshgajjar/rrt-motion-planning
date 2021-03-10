import unittest

import numpy as np

from collision import CollisionSphere, CollisionBox
from planar_rrt import PlanarRRT


class TestPlanarRRT(unittest.TestCase):
    def test_build_with_obstacles(self):
        num_retries = 3
        for _ in range(num_retries):
            obstacle_list = [
                CollisionSphere((0.5, 0.5), 0.1),
                CollisionBox((0.5, 0.25), (0.1, 0.25)),
            ]
            rrt1 = PlanarRRT(
                start_state=(0.2, 0.2),
                goal_state=(0.8, 0.8),
                dim_ranges=[(0, 1)] * 2,
                max_iter=500,
                obstacles=obstacle_list,
                visualize=True,)
            path = rrt1.build()
            if path is not None:
                break
            else:
                print("retrying...")

        self.assertIsNotNone(path)
        self.assertTrue(np.all(path[0] == [0.2, 0.2]))
        self.assertTrue(np.all(path[-1] == [0.8, 0.8]))

        # Ensure RRT did not grow into collision objects
        for node in rrt1.start:
            for obj in obstacle_list:
                self.assertFalse(obj.in_collision(node.state))
