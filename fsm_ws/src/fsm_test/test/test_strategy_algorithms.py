import unittest

from wall_destacking_strategy.algorithms import AABB, assign_grid_indices_by_yz, fit_wall_plane_ransac, point_in_aabb


class TestStrategyAlgorithms(unittest.TestCase):
    def test_l0_map_01_ransac_wall_plane_fit_rejects_outliers(self):
        points = []
        for row in range(5):
            for col in range(5):
                noise = 0.002 * (((row + col) % 3) - 1)
                points.append((0.6 + noise, (2 - col) * 0.4, 1.8 - row * 0.4))
        points.extend([(1.3, 2.0, 2.0), (-0.2, -2.0, 0.0)])

        result = fit_wall_plane_ransac(points, distance_threshold=0.01, min_inliers=20)

        self.assertGreaterEqual(len(result.inlier_indices), 25)
        self.assertGreater(result.normal[0], 0.99)
        self.assertLess(abs(result.normal[1]), 0.05)
        self.assertLess(abs(result.normal[2]), 0.05)
        self.assertAlmostEqual(result.centroid[0], 0.6, places=2)
        self.assertGreater(result.confidence, 0.8)

    def test_l0_map_02_assign_grid_indices_by_yz(self):
        points = [
            (0.6, (2 - col) * 0.4, 1.8 - row * 0.4)
            for row in range(5)
            for col in range(5)
        ]
        shuffled = [points[index] for index in (12, 0, 24, 4, 20, 6, 18, 1, 23, 2, 7, 3, 5, 8, 9, 10, 11, 13, 14, 15, 16, 17, 19, 21, 22)]

        assignments = assign_grid_indices_by_yz(shuffled, rows=5, cols=5)
        by_row_col = {(item.row, item.col): item.point for item in assignments}

        self.assertEqual(len(assignments), 25)
        self.assertEqual(by_row_col[(0, 0)], (0.6, 0.8, 1.8))
        self.assertEqual(by_row_col[(0, 4)], (0.6, -0.8, 1.8))
        self.assertAlmostEqual(by_row_col[(4, 0)][2], 0.2)
        self.assertAlmostEqual(by_row_col[(4, 4)][2], 0.2)
        self.assertEqual(by_row_col[(4, 0)][1], 0.8)
        self.assertEqual(by_row_col[(4, 4)][1], -0.8)

    def test_l0_pair_aabb_reachability_margin(self):
        workspace = AABB(x_min=0.3, x_max=0.9, y_min=-0.1, y_max=0.6, z_min=0.3, z_max=1.8)

        self.assertTrue(point_in_aabb((0.6, 0.4, 1.0), workspace))
        self.assertFalse(point_in_aabb((0.6, 0.8, 1.0), workspace, margin=0.0))
        self.assertTrue(point_in_aabb((0.6, 0.8, 1.0), workspace, margin=0.4))
