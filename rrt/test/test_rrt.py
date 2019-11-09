import unittest

import numpy as np

from collision import CollisionSphere, CollisionBox
from rrt import RRT


class TestRRTInit(unittest.TestCase):
    def test_rrt_init_basic(self):
        rrt = RRT(
            start_state=(0, 0),
            goal_state=(1, 1),
            dim_ranges=[(-2, 2), (-2, 2)])

    def test_rrt_init_args(self):
        rrt = RRT(
            start_state=(0, 0),
            goal_state=(1, 1),
            dim_ranges=[(-2, 2), (-2, 2)],
            obstacles=None,
            step_size=0.1,
            max_iter=1000)

    def test_rrt_init_bad_dims(self):
        with self.assertRaises(AssertionError):
            rrt = RRT(
                start_state=(0, 0),
                goal_state=(1, 1, 1),
                dim_ranges=[(-2, 2), (-2, 2)])


class TestRRTSteps(unittest.TestCase):
    def setUp(self):
        # RRT 1: RRT with 1 node, in 3 dimensions
        self.rrt1 = RRT(
            start_state=(0, 0, 0),
            goal_state=(1, 1, 1),
            dim_ranges=[(-2, 2)] * 3,
            step_size=0.1)

        # RRT 2: RRT with 2 nodes, in 2 dimensions
        # (start) --> (node1)
        self.rrt2 = RRT(
            start_state=(0, 0),
            goal_state=(1, 1),
            dim_ranges=[(-2, 2), (-2, 2)],
            step_size=0.05)
        self.rrt2_node1 = self.rrt2.start.add_child(state=(0.5, 0.25))

        # RRT 3: RRT with 2 nodes, in 3 dimensions
        self.rrt3 = RRT(
            start_state=(0.25, 0.25, 0.25),
            goal_state=(0.75, 0.75, 0.75),
            dim_ranges=[(0, 1)] * 3,
            step_size=0.1)
        self.rrt3_node1 = self.rrt3.start.add_child(state=(0.5, 0.5, 0.5))
        self.rrt3_node2 = self.rrt3_node1.add_child(state=(0.75, 0.75, 0.7))

    def test_get_random_sample(self):
        sample = self.rrt1._get_random_sample()
        lower = np.array([d[0] for d in self.rrt1.dim_ranges])
        upper = np.array([d[1] for d in self.rrt1.dim_ranges])

        self.assertEqual(len(sample), len(self.rrt1.dim_ranges))
        self.assertTrue(np.all(sample >= lower))
        self.assertTrue(np.all(sample < upper))

    def test_get_nearest_neighbor(self):
        sample1 = np.array([0.1, 0.2, 0.3])
        neighbor1 = self.rrt1._get_nearest_neighbor(sample1)
        self.assertIs(neighbor1, self.rrt1.start)

        sample2 = np.array([0.25, 0.25])
        neighbor2 = self.rrt2._get_nearest_neighbor(sample2)
        self.assertIs(neighbor2, self.rrt2_node1)

        sample3 = np.array([0.7, 0.7, 0.7])
        neighbor3 = self.rrt3._get_nearest_neighbor(sample3)
        self.assertIs(neighbor3, self.rrt3_node2)

    def test_extend_sample(self):
        sample1 = np.array([0, 0, 0.1])
        sample1_node = self.rrt1._extend_sample(sample1, self.rrt1.start)

        self.assertIs(sample1_node.parent, self.rrt1.start)
        self.assertAlmostEqual(
            np.linalg.norm(sample1 - sample1_node.state),
            0.,
            places=3)

        sample2 = np.array([1, 1, 1])
        sample2_ext = np.array([0.0577, 0.0577, 0.0577])
        sample2_node = self.rrt1._extend_sample(sample2, self.rrt1.start)
        self.assertAlmostEqual(
            np.linalg.norm(sample2_ext - sample2_node.state),
            0.,
            places=3)

    def test_check_for_completion(self):
        complete1 = self.rrt2._check_for_completion(self.rrt2_node1)
        self.assertFalse(complete1)

        complete2 = self.rrt3._check_for_completion(self.rrt3_node2)
        self.assertTrue(complete2)

    def test_trace_path_from_start(self):
        path1 = self.rrt1._trace_path_from_start(self.rrt1.start)
        # print(path1)
        self.assertEqual(len(path1), 1)
        self.assertTrue(np.all(path1[0] == self.rrt1.start.state))

        path2 = self.rrt2._trace_path_from_start(self.rrt2_node1)
        self.assertEqual(len(path2), 2)
        self.assertTrue(np.all(path2[0] == self.rrt2.start.state))
        self.assertTrue(np.all(path2[1] == self.rrt2_node1.state))

        self.rrt3_node2.children.append(self.rrt3.goal)
        self.rrt3.goal.parent = self.rrt3_node2
        path3 = self.rrt3._trace_path_from_start()
        self.assertEqual(len(path3), 4)
        self.assertTrue(np.all(path3[0] == self.rrt3.start.state))
        self.assertTrue(np.all(path3[1] == self.rrt3_node1.state))
        self.assertTrue(np.all(path3[2] == self.rrt3_node2.state))
        self.assertTrue(np.all(path3[3] == self.rrt3.goal.state))

    def test_check_for_collision(self):
        sample = np.array([0.5, 0.5, 0.5])
        in_collision = self.rrt1._check_for_collision(sample)
        self.assertFalse(in_collision)


class TestCollisionObjs(unittest.TestCase):
    def test_collision_circle(self):
        obj1 = CollisionSphere([0, 0], 1)
        p1, p2, p3 = [0, 0], [1, 2], [0, 1]
        self.assertTrue(obj1.in_collision(p1))
        self.assertFalse(obj1.in_collision(p2))
        self.assertTrue(obj1.in_collision(p3))

        obj2 = CollisionSphere([1, 2, 3, 4], 6)
        p4 = [1, -1, 1, -1]
        self.assertFalse(obj2.in_collision(p4))

    def test_collision_box(self):
        obj1 = CollisionBox([0, 0], [0.5, 0.5])
        p1, p2, p3 = [0, 0], [1, 0], [0, 0.5]
        self.assertTrue(obj1.in_collision(p1))
        self.assertFalse(obj1.in_collision(p2))
        self.assertTrue(obj1.in_collision(p3))

        obj2 = CollisionBox([1, 2, 3, 4], [4, 3, 2, 1])
        p4 = [1, -1, 1, -1]
        self.assertFalse(obj2.in_collision(p4))


class TestRRTBuild(unittest.TestCase):
    def test_build(self):
        num_retries = 3
        for _ in range(num_retries):
            rrt1 = RRT(
                start_state=(0.2, 0.2),
                goal_state=(0.8, 0.8),
                dim_ranges=[(0, 1), (0, 1)],
                max_iter=1000)
            path = rrt1.build()
            if path is not None:
                break
            else:
                print("retrying...")

        self.assertIsNotNone(path)
        self.assertTrue(np.all(path[0] == [0.2, 0.2]))
        self.assertTrue(np.all(path[-1] == [0.8, 0.8]))


if __name__ == '__main__':
    unittest.main()
