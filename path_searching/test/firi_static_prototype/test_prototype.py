#!/usr/bin/env python3
"""Run with python3 -m unittest discover -s <this directory> -v."""
import time
import unittest
import numpy as np
from prototype import (Grid, Config, GeometryFailure, corridor, inflate,
                       coverage, area, overlap_certificate)


def empty():
    return Grid(np.zeros((24, 48), dtype=int), .25, np.zeros(2))


class PrototypeTests(unittest.TestCase):
    def test_dense_straight_independent(self):
        g = empty()
        sparse, _ = corridor(g, [[1, 3], [11, 3]])
        dense, _ = corridor(g, np.column_stack((np.linspace(1, 11, 501), np.full(501, 3.))))
        self.assertEqual(len(sparse), len(dense))
        self.assertLessEqual(len(sparse), 4)
        self.assertGreater(min(area(r.vertices) for r in sparse), 10.)

    def test_corners(self):
        for path in ([[1, 1], [1, 5], [10, 5]],
                     [[1, 1], [5, 5], [10, 1]],
                     [[1, 1], [4, 1], [4, 5], [8, 5], [8, 1], [11, 1]]):
            with self.subTest(path=path):
                regions, overlaps = corridor(empty(), path)
                self.assertGreater(len(regions), 1)
                for a, d in overlaps:
                    self.assertGreaterEqual(a, .002)
                    self.assertGreaterEqual(d, .05-1e-8)

    def test_obstacle_route_and_certificate(self):
        g = empty(); g.values[6:17, 20:24] = 1
        regions, overlaps = corridor(g, [[1, 2], [4, 2], [4, 5], [8, 5], [11, 2]])
        self.assertTrue(overlaps)
        self.assertTrue(any(len(r.vertices) > 4 for r in regions))
        for r in regions:
            _, _, cells = g.local_geometry(r.seed, 3.)
            for v in cells:
                self.assertTrue(np.any(np.min(r.A@v.T-r.b[:, None], axis=1) >= .002-1e-7))

    def test_near_wall_and_narrow_feasible(self):
        g = empty(); g.values[:10] = 1; g.values[13:] = 1
        regions, overlaps = corridor(g, [[1, 2.65], [11, 2.65]])
        self.assertTrue(regions)
        self.assertTrue(all(d >= .05-1e-8 for _, d in overlaps))

    def test_narrow_insufficient_overlap(self):
        g = Grid(np.ones((10, 220), int), .05, np.zeros(2))
        g.values[4] = 0
        with self.assertRaises(GeometryFailure):
            corridor(g, [[.5, .225], [10, .225]])

    def test_cell_crossing_without_center_hit(self):
        g = empty(); g.values[8, 20] = 1
        # Crosses cell near its bottom edge, never crosses its center.
        with self.assertRaises(GeometryFailure):
            corridor(g, [[1, 2.02], [11, 2.02]])

    def test_unknown_and_map_edge(self):
        g = empty(); g.values[:, 20] = -1
        with self.assertRaises(GeometryFailure):
            corridor(g, [[1, 3], [11, 3]])
        for p in ([[-.1, 1], [3, 1]], [[1, 1], [13, 1]], [[0, 1], [1, 1]]):
            with self.subTest(p=p), self.assertRaises(GeometryFailure):
                corridor(empty(), p)

    def test_invalid(self):
        for path in ([], [[1, 1]], [[1, 1], [1, 1]], [[1, 1], [np.nan, 2]]):
            with self.subTest(path=path), self.assertRaises(GeometryFailure):
                corridor(empty(), path)
        for cfg in (Config(local_radius=np.nan), Config(iterations=0),
                    Config(overlap_depth=-1), Config(clearance=1)):
            with self.assertRaises(GeometryFailure):
                corridor(empty(), [[1, 1], [2, 1]], cfg)

    def test_budgets(self):
        with self.assertRaisesRegex(GeometryFailure, 'REGION_BUDGET'):
            corridor(empty(), [[1, 3], [11, 3]], Config(max_regions=1))
        with self.assertRaisesRegex(GeometryFailure, 'TIME_BUDGET'):
            corridor(empty(), [[1, 3], [11, 3]], Config(max_seconds=1e-12))
        with self.assertRaisesRegex(GeometryFailure, 'NO_PROGRESS'):
            corridor(empty(), [[1, 3], [11, 3]], Config(min_progress=10))

    def test_continuous_edges_no_cut_corner(self):
        r = inflate(empty(), [1, 1], Config(local_radius=1.5))
        path = np.array([[1., 1.], [4., 1.], [1., 1.]])
        arc = np.array([0., 3., 6.])
        end = coverage(r, path, arc, 0.)
        self.assertAlmostEqual(end, 1.498, places=6)
        self.assertLess(end, 3.)  # returning into cell never bridges a gap

    def test_seed_and_ellipse_certificates(self):
        g = empty(); g.values[10:14, 16:20] = 1
        r = inflate(g, [3, 3])
        self.assertLessEqual(np.max(r.A@r.seed-r.b), 1e-8)
        self.assertGreaterEqual(np.min(r.b-r.A@r.ellipsoid_center-
                                      np.linalg.norm(r.A@r.ellipsoid_L, axis=1)), -1e-8)
        with self.assertRaisesRegex(GeometryFailure, 'SEED_BLOCKED'):
            inflate(g, [4.1, 3])

    def test_optimizer_failure_is_not_success(self):
        g = empty(); g.values[6:17, 20:24] = 1
        with self.assertRaisesRegex(GeometryFailure, 'SOLVER_FAILURE'):
            inflate(g, [3, 2], Config(optimizer_iterations=1))

    def test_invalid_grid(self):
        for g in (Grid(np.zeros((0, 2)), .25, np.zeros(2)),
                  Grid(np.zeros((2, 2)), np.nan, np.zeros(2)),
                  Grid(np.full((2, 2), 2), .25, np.zeros(2))):
            with self.assertRaisesRegex(GeometryFailure, 'INVALID_GRID'):
                inflate(g, [.1, .1])

    def test_translation_stable_area(self):
        v = np.array([[0., 0.], [3.098, 0.], [3.098, 5.996], [0., 5.996]])
        for offset in (0., 9000., 1e9):
            translated = v+offset
            # At 1e9 the input vertices themselves have quantization error,
            # but area must not suffer absolute-product cancellation.
            self.assertAlmostEqual(area(translated), area(v), delta=2e-6)
            self.assertLess(area(translated), 100.)
            self.assertEqual(area(np.array([[0., 0.], [1., 1.], [2., 2.]])+offset), 0.)

    def test_translated_corridor_valid_and_unattainable_area(self):
        base, base_overlaps = corridor(empty(), [[1., 3.], [11., 3.]])
        for offset in (-9000., 9000.):
            g = empty(); g.origin = np.full(2, offset)
            path = np.array([[1., 3.], [11., 3.]])+offset
            regions, overlaps = corridor(g, path)
            self.assertEqual(len(regions), len(base))
            np.testing.assert_allclose(overlaps, base_overlaps, atol=1e-6, rtol=0)
            with self.assertRaisesRegex(GeometryFailure, 'INSUFFICIENT_OVERLAP'):
                corridor(g, path, Config(overlap_area=100.))

    def test_unsupported_scale_reproduction(self):
        g = empty(); g.origin = np.full(2, 1e9)
        path = np.array([[1., 3.], [11., 3.]])+1e9
        for cfg in (Config(), Config(overlap_area=100.)):
            with self.assertRaisesRegex(GeometryFailure, 'UNSUPPORTED_NUMERICAL_SCALE'):
                corridor(g, path, cfg)
        for g in (Grid(np.zeros((2, 2)), .001, np.zeros(2)),
                  Grid(np.zeros((2, 2)), 100., np.zeros(2)),
                  Grid(np.zeros((2, 2)), .25, np.full(2, 1e9))):
            with self.assertRaisesRegex(GeometryFailure, 'UNSUPPORTED_NUMERICAL_SCALE'):
                g.validate()
        with self.assertRaisesRegex(GeometryFailure, 'UNSUPPORTED_NUMERICAL_SCALE'):
            corridor(empty(), [[1, 1], [2, 1]], Config(clearance=1e-10))

    def test_runtime(self):
        g = empty(); g.values[6:17, 20:24] = 1
        timings = []
        for _ in range(5):
            t = time.monotonic()
            regions, _ = corridor(g, [[1, 2], [4, 2], [4, 5], [8, 5], [11, 2]])
            timings.append(time.monotonic()-t)
        print('\nobstacle-route seconds:', timings, 'regions:', len(regions), flush=True)
        self.assertLess(max(timings), 15.)


if __name__ == '__main__':
    unittest.main(verbosity=2)
