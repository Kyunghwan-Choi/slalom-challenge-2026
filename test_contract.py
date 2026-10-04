"""Small API/geometry checks, independent of the PyChrono installation."""

import json
import math
import unittest
from pathlib import Path

import numpy as np

from contract import (
    acceleration_bounds,
    apply_action,
    apply_easy_action,
    footprint_clearance,
    gate_side_valid,
    inside_road,
    pedal_acceleration,
    scenario_from_config,
)
from dynamics import derivative, parameters, predict

CFG = json.loads(Path(__file__).with_name("course.json").read_text(encoding="utf-8"))


class ContractTests(unittest.TestCase):
    def test_rate_limit_and_pedal_split(self):
        brake_ax = pedal_acceleration(5.0, -0.6, CFG)
        drive_ax = pedal_acceleration(5.0, 0.3, CFG)
        np.testing.assert_allclose(
            apply_action({"steering": 0.8, "acceleration": brake_ax}, 0.0, CFG, 5.0),
            [0.04, 0.0, 0.6],
            atol=1e-12,
        )
        np.testing.assert_allclose(
            apply_action({"steering": -0.8, "acceleration": drive_ax}, 0.04, CFG, 5.0),
            [0.0, 0.3, 0.0],
            atol=1e-12,
        )
        np.testing.assert_allclose(
            apply_easy_action({"steering": 0.8, "longitudinal": -0.6}, 0.0, CFG), [0.04, 0.0, 0.6]
        )

    def test_acceleration_feasibility_and_inverse(self):
        lower, upper = acceleration_bounds(10.0, CFG)
        self.assertLess(lower, -5.5)
        self.assertGreater(upper, 2.0)
        self.assertLess(upper, 2.8)
        self.assertGreater(acceleration_bounds(11.0, CFG)[1], 2.0)
        for speed in (0.0, 0.2, 2.0, 5.0, 10.0, 11.0, 12.0):
            for pedal in (-0.6, -0.3, 0.0, 0.2, 0.4, 0.6):
                requested = pedal_acceleration(speed, pedal, CFG)
                applied = apply_action(
                    {"steering": 0.0, "acceleration": requested}, 0.0, CFG, speed
                )
                achieved = pedal_acceleration(speed, applied[1] - applied[2], CFG)
                self.assertAlmostEqual(achieved, requested, places=10)
        high = apply_action({"steering": 0.0, "acceleration": 7.0}, 0.0, CFG, 11.0)
        self.assertAlmostEqual(high[1], CFG["pedal_limit"])
        self.assertAlmostEqual(
            pedal_acceleration(11.0, high[1], CFG), acceleration_bounds(11.0, CFG)[1]
        )
        stopped = apply_action({"steering": 0.0, "acceleration": -7.0}, 0.0, CFG, 0.0)
        self.assertEqual(stopped[1:], [0.0, 0.0])
        reversing = apply_action({"steering": 0.0, "acceleration": -7.0}, 0.0, CFG, -0.2)
        self.assertEqual(reversing[1:], [0.0, CFG["pedal_limit"]])

    def test_invalid_actions(self):
        for value in [float("nan"), float("inf"), True, 0.81, "0.2"]:
            with self.subTest(value=value), self.assertRaises(ValueError):
                apply_action({"steering": value, "acceleration": 0.0}, 0.0, CFG, 5.0)
        with self.assertRaises(ValueError):
            apply_action({"steering": 0.0, "acceleration": 0.0, "throttle": 0.1}, 0.0, CFG, 5.0)
        for value in (float("nan"), float("inf"), True, 7.1, "1.0"):
            with self.subTest(acceleration=value), self.assertRaises(ValueError):
                apply_action({"steering": 0.0, "acceleration": value}, 0.0, CFG, 5.0)

    def test_high_speed_adapter_matches_predictor(self):
        p = parameters(0.9)
        for speed in (9.0, 9.2, 9.35, 9.5, 10.0, 11.0, 11.5):
            lower, upper = acceleration_bounds(speed, CFG)
            for request in (-7.0, -3.0, 0.0, 2.0, 7.0):
                with self.subTest(speed=speed, request=request):
                    u = apply_action({"steering": 0.0, "acceleration": request}, 0.0, CFG, speed)
                    achieved = pedal_acceleration(speed, u[1] - u[2], CFG)
                    self.assertAlmostEqual(achieved, max(lower, min(upper, request)), places=10)
                    s = np.array([0.0, 0.0, 0.0, speed, 0.0, 0.0, 0.0])
                    self.assertAlmostEqual(
                        achieved, derivative(s, np.array(u), 0.9, p)[3], places=10
                    )

    def test_rectangular_geometry(self):
        state = [1.371, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0]
        self.assertAlmostEqual(footprint_clearance(state, {"x": 0.0, "y": 1.0}, CFG), -0.20)
        self.assertAlmostEqual(footprint_clearance(state, {"x": 3.35, "y": 0.0}, CFG), 0.75)
        self.assertTrue(inside_road([0.0, 4.5, 0.0], CFG))
        self.assertFalse(inside_road([0.0, 4.6, 0.0], CFG))
        self.assertFalse(inside_road([0.0, 4.6, math.pi / 2], CFG))

    def test_map_and_model_adapter(self):
        scenario = scenario_from_config(CFG)
        self.assertEqual(
            [c["x"] for c in scenario["cones"]], [20.0, 35.0, 50.0, 65.0, 80.0, 95.0, 110.0, 125.0]
        )
        self.assertEqual([c["pass_sign"] for c in scenario["cones"]], [1, -1, 1, -1, 1, -1, 1, -1])
        ax = pedal_acceleration(5.0, 0.2, CFG)
        s, u = predict(
            [0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0], {"steering": 0.8, "acceleration": ax}, 0.0, CFG
        )
        self.assertEqual(s.shape, (7,))
        self.assertTrue(np.isfinite(s).all())
        self.assertGreater(s[0], 0.0)
        self.assertGreater(s[6], 0.0)
        np.testing.assert_allclose(u, [0.04, 0.2, 0.0])
        with self.assertRaises(ValueError):
            parameters(0.4)

    def test_custom_cone_centers_keep_alternating_gates(self):
        cfg = dict(
            CFG,
            cone_centers_m=[[20.0 + 16 * j, (-1.0) ** j * 0.2] for j in range(8)],
            finish_x_m=152.0,
        )
        cones = scenario_from_config(cfg)["cones"]
        self.assertEqual(cones[2], {"x": 52.0, "y": 0.2, "pass_sign": 1})
        with self.assertRaisesRegex(ValueError, "increase"):
            scenario_from_config(dict(cfg, cone_centers_m=[[20.0, 0.0]] * 8))

    def test_gate_requires_correct_half_plane_without_fixed_offset(self):
        plus = {"x": 20.0, "y": 0.2, "pass_sign": 1}
        minus = {"x": 35.0, "y": -0.2, "pass_sign": -1}
        self.assertTrue(gate_side_valid(0.200001, plus))
        self.assertTrue(gate_side_valid(-0.200001, minus))
        self.assertFalse(gate_side_valid(0.2, plus))
        self.assertFalse(gate_side_valid(-0.1, minus))


if __name__ == "__main__":
    unittest.main()
