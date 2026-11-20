import unittest
from math import hypot, sqrt

from algorithms.nominal_ds import (
    cap_velocity,
    compute_velocity,
    modulate_velocity_ellipsoid,
    nominal_path,
)


class NominalDSTests(unittest.TestCase):
    def test_velocity_points_toward_goal_and_respects_speed_limit(self):
        velocity = compute_velocity((0.0, 0.0), (3.0, 4.0), 1.0, 0.5)
        self.assertAlmostEqual(velocity[0], 0.3)
        self.assertAlmostEqual(velocity[1], 0.4)

    def test_three_dimensional_velocity_respects_speed_limit(self):
        velocity = compute_velocity((0.0, 0.0, 0.0), (1.0, 2.0, 2.0), 1.0, 0.5)
        self.assertAlmostEqual(velocity[0], 1.0 / 6.0)
        self.assertAlmostEqual(velocity[1], 1.0 / 3.0)
        self.assertAlmostEqual(velocity[2], 1.0 / 3.0)

    def test_ellipsoid_modulation_reduces_inward_normal_velocity(self):
        velocity = modulate_velocity_ellipsoid(
            (1.5, 0.0, 0.0),
            (-1.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (1.0, 1.0, 1.0),
        )
        self.assertAlmostEqual(velocity[0], -5.0 / 9.0)
        self.assertAlmostEqual(velocity[1], 0.0)
        self.assertAlmostEqual(velocity[2], 0.0)

    def test_ellipsoid_modulation_preserves_outward_normal_velocity_without_tail(self):
        velocity = modulate_velocity_ellipsoid(
            (1.5, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (1.0, 1.0, 1.0),
        )
        self.assertAlmostEqual(velocity[0], 1.0)

    def test_ellipsoid_modulation_rejects_position_at_obstacle_center(self):
        with self.assertRaises(ValueError):
            modulate_velocity_ellipsoid(
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
                (1.0, 1.0, 1.0),
            )

    def test_modulated_current_route_reaches_goal_and_respects_clearance(self):
        position = (0.403, 0.587, 0.543)
        goal = (0.403, -0.463, 0.643)
        center = (0.65, -0.24, 0.60)
        obstacle_size = (0.70, 0.10, 0.20)
        safety_margin = 0.05
        semi_axes = tuple(
            sqrt(3.0) * dimension / 2.0 + safety_margin
            for dimension in obstacle_size
        )
        minimum_gamma = float("inf")

        for _ in range(3000):
            if hypot(*(target - current for current, target in zip(position, goal))) <= 0.005:
                break
            nominal = compute_velocity(position, goal, 0.75, 0.10)
            modulated = modulate_velocity_ellipsoid(
                position, nominal, center, semi_axes
            )
            velocity = cap_velocity(modulated, 0.10)
            position = tuple(
                current + 0.02 * speed
                for current, speed in zip(position, velocity)
            )
            gamma = sum(
                ((current - obstacle_center) / axis) ** 2
                for current, obstacle_center, axis in zip(position, center, semi_axes)
            )
            minimum_gamma = min(minimum_gamma, gamma)
        else:
            self.fail("Modulated path did not reach the configured goal.")

        self.assertLessEqual(
            hypot(*(target - current for current, target in zip(position, goal))),
            0.005,
        )
        self.assertGreaterEqual(minimum_gamma, 1.0)

    def test_path_ends_at_goal(self):
        path = nominal_path(
            (0.0, 0.0), (0.1, 0.0), gain=0.5, speed_limit=0.04,
            dt=0.02, tolerance=0.005, maximum_steps=3000,
        )
        self.assertEqual(path[-1], (0.1, 0.0))
        self.assertTrue(all(point[1] == 0.0 for point in path))

    def test_path_fails_if_step_limit_is_too_small(self):
        with self.assertRaises(RuntimeError):
            nominal_path(
                (0.0, 0.0), (10.0, 0.0), gain=0.5, speed_limit=0.04,
                dt=0.02, tolerance=0.005, maximum_steps=1,
            )


if __name__ == "__main__":
    unittest.main()
