"""Public easy-mode plan validator and fixed PyChrono action tracker.

This module contains no DP solver or successful plan. A student supplies moves.
"""
from pathlib import Path
import bisect
import json
import math
from contract import gate_side_valid


SPEC_FILE = Path(__file__).with_name("easy_spec.json")


def specification():
    return json.loads(SPEC_FILE.read_text(encoding="utf-8"))


def _motion(m, n, spec):
    dx = m * spec["grid_x_m"]
    dy = n * spec["grid_y_m"]
    return math.hypot(dx, dy) / spec["planning_dt_s"], math.atan2(dy, dx)


def validate_plan(plan, scenario, spec=None):
    """Return the grid nodes; raise ValueError on any inadmissible move."""
    spec = specification() if spec is None else spec
    if not isinstance(plan, dict) or set(plan) != {"moves"} or not isinstance(plan["moves"], list):
        raise ValueError('Easy plan must be a JSON object with exactly one key: "moves"')
    moves = plan["moves"]
    if not 1 <= len(moves) <= spec["maximum_stages"]:
        raise ValueError("Easy plan length must be between 1 and maximum_stages")
    x_index = y_index = 0
    old_v = old_chi = 0.0
    nodes = [{"X": 0.0, "Y": 0.0, "V": 0.0, "chi": 0.0}]
    finish = scenario["finish_x_m"]
    for stage, move in enumerate(moves, 1):
        if (not isinstance(move, list) or len(move) != 2 or
            any(isinstance(q, bool) or not isinstance(q, int) for q in move)):
            raise ValueError(f"Move {stage}: expected [integer m, integer n]")
        m, n = move
        if m not in spec["longitudinal_moves"] or not spec["lateral_move_min"] <= n <= spec["lateral_move_max"]:
            raise ValueError(f"Move {stage}: motion index outside the action lattice")
        nx = x_index + m
        ny = y_index + n
        if abs(ny * spec["grid_y_m"]) > spec["maximum_abs_y_m"] + 1e-12:
            raise ValueError(f"Move {stage}: outside planning road strip")
        v, chi = _motion(m, n, spec)
        if v > spec["maximum_segment_speed_m_s"] + 1e-12 or abs(chi) > spec["maximum_abs_segment_course_angle_rad"] + 1e-12:
            raise ValueError(f"Move {stage}: virtual speed or course angle exceeds limit")
        dt = spec["planning_dt_s"]
        dchi = abs(chi-old_chi)
        if (v-old_v > spec["maximum_acceleration_m_s2"]*dt + 1e-12 or
            old_v-v > spec["maximum_deceleration_m_s2"]*dt + 1e-12 or
            dchi > spec["maximum_course_angle_change_rad"] + 1e-12 or
            v*dchi/dt > spec["maximum_virtual_lateral_acceleration_m_s2"] + 1e-12):
            raise ValueError(f"Move {stage}: violates speed or turning transition limit")
        x0 = x_index * spec["grid_x_m"]
        x1 = nx * spec["grid_x_m"]
        for cone in scenario["cones"]:
            if x0 < cone["x"] <= x1:
                fraction = (cone["x"]-x0)/(x1-x0)
                gate_y = (y_index+fraction*n)*spec["grid_y_m"]
                if not gate_side_valid(gate_y, cone):
                    raise ValueError(f"Move {stage}: fails planned gate {cone['x']} m")
        nodes.append({"X": x1, "Y": ny*spec["grid_y_m"], "V": v, "chi": chi})
        x_index, y_index = nx, ny
        old_v, old_chi = v, chi
        if x1 >= finish and stage != len(moves):
            raise ValueError("The final move must be the first arrival at the finish")
    if nodes[-1]["X"] < finish:
        raise ValueError("Easy plan does not reach the finish")
    return nodes


def load_plan(path, scenario):
    return validate_plan(json.loads(Path(path).read_text(encoding="utf-8")), scenario)


def _clip(value, lower, upper):
    return max(lower, min(upper, value))


class EasyTracker:
    """Fixed lower controller shared by every easy-mode plan."""

    def __init__(self, nodes):
        self.x = [p["X"] for p in nodes]
        self.y = [p["Y"] for p in nodes]
        self.v = [p["V"] for p in nodes]

    def reset(self, scenario, seed):
        pass

    def _interpolate(self, values, x):
        i = max(0, min(len(self.x)-2, bisect.bisect_right(self.x, x)-1))
        ratio = _clip((x-self.x[i])/(self.x[i+1]-self.x[i]), 0.0, 1.0)
        return values[i] + ratio*(values[i+1]-values[i])

    def act(self, observation):
        x, y, psi, vx, vy, r, delta = observation["state"]
        lookahead = 3.0
        origin_x = x - .8*math.cos(psi)
        origin_y = y - .8*math.sin(psi)
        dy = self._interpolate(self.y, origin_x+lookahead)-origin_y
        local_x = math.cos(psi)*lookahead + math.sin(psi)*dy
        local_y = -math.sin(psi)*lookahead + math.cos(psi)*dy
        steer = _clip(.85*math.atan2(2*2.776*local_y, local_x**2+local_y**2)/.626671, -.8, .8)
        target_v = self._interpolate(self.v, x+2.5)
        error = target_v-vx
        pedal = _clip(.12+.35*error, 0., .6) if error >= 0 else -_clip(-.3*error, 0., .6)
        return {"steering": steer, "longitudinal": pedal}
