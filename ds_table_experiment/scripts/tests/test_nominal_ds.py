import unittest

from algorithms.nominal_ds import compute_velocity, nominal_path


class NominalDSTests(unittest.TestCase):
    def test_velocity_points_toward_goal_and_respects_speed_limit(self):
        velocity = compute_velocity((0.0, 0.0), (3.0, 4.0), 1.0, 0.5)
        self.assertAlmostEqual(velocity[0], 0.3)
        self.assertAlmostEqual(velocity[1], 0.4)

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
