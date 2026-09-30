"""Check the public easy-mode plan contract without publishing a course solution."""
import unittest

from easy_mode import validate_plan


class EasyModeContractTest(unittest.TestCase):
    def setUp(self):
        self.short = {"finish_x_m": 1.0, "cones": []}

    def test_one_stage_first_arrival(self):
        nodes = validate_plan({"moves": [[1, 0]]}, self.short)
        self.assertEqual(nodes[-1]["X"], 1.0)
        self.assertEqual(nodes[-1]["V"], 2.0)

    def test_early_finish_followed_by_extra_move_is_invalid(self):
        with self.assertRaisesRegex(ValueError, "first arrival"):
            validate_plan({"moves": [[1, 0], [1, 0]]}, self.short)

    def test_gate_is_checked_at_segment_crossing(self):
        scenario = {"finish_x_m": 2.0, "cones": [{"x": 1.0, "y": 0.0, "pass_sign": 1}]}
        with self.assertRaisesRegex(ValueError, "gate"):
            validate_plan({"moves": [[1, 0], [1, 0]]}, scenario)

    def test_unreachable_initial_acceleration_is_invalid(self):
        with self.assertRaisesRegex(ValueError, "transition"):
            validate_plan({"moves": [[5, 0]]}, {"finish_x_m": 5.0, "cones": []})

    def test_small_correct_side_crossing_is_planning_valid(self):
        scenario = {"finish_x_m": 5.0, "cones": [{"x": 5.0, "y": 0.0, "pass_sign": 1}]}
        nodes = validate_plan({"moves": [[1, 0], [2, 0], [2, 1]]}, scenario)
        self.assertEqual(nodes[-1]["Y"], .5)


if __name__ == "__main__":
    unittest.main()
