"""Local practice runner, headless by default and with an optional live viewer."""

import argparse
import copy
import importlib.util
import json
import math
import sys
import time
from pathlib import Path

from contract import (
    apply_action,
    apply_easy_action,
    footprint_clearance,
    gate_side_valid,
    inside_road,
    scenario_from_config,
)


def run(
    controller_path, config_path, output, seed=0, mode="full", plan_path=None, show_visual=False
):
    import numpy as np
    import pychrono as c
    import pychrono.vehicle as v

    cfg = json.loads(Path(config_path).read_text(encoding="utf-8"))
    scenario = scenario_from_config(cfg)
    if mode not in ("easy", "full"):
        raise ValueError("mode must be 'easy' or 'full'")
    scenario["mode"] = mode
    if mode == "easy":
        if plan_path is None:
            raise ValueError("--plan is required in easy mode")
        from easy_mode import EasyTracker, load_plan, specification

        scenario["easy_dp"] = specification()
        plan_nodes = load_plan(plan_path, scenario)
        planned_stages = len(plan_nodes) - 1
    else:
        plan_nodes = None
        planned_stages = None
    car = v.BMW_E90()
    car.SetContactMethod(c.ChContactMethod_SMC)
    car.SetChassisCollisionType(v.CollisionType_PRIMITIVES)
    car.SetInitPosition(c.ChCoordsysd(c.ChVector3d(0, 0, 0.6), c.ChQuaterniond(1, 0, 0, 0)))
    car.SetInitFwdVel(0.0)
    car.SetInitWheelAngVel([0.0] * 4)
    car.SetTireType(v.TireModelType_TMEASY)
    car.SetTireStepSize(cfg["physics_dt_s"])
    car.Initialize()
    system = car.GetSystem()
    system.SetCollisionSystemType(c.ChCollisionSystem.Type_BULLET)
    terrain = v.RigidTerrain(system)
    mat = c.ChContactMaterialSMC()
    mat.SetFriction(cfg["friction"])
    mat.SetYoungModulus(2e7)
    ground = terrain.AddPatch(
        mat, c.ChCoordsysd(c.ChVector3d(100, 0, 0), c.ChQuaterniond(1, 0, 0, 0)), 500, 100
    )
    terrain.Initialize()
    ids = set()
    cone_bodies = []
    cone_mat = c.ChContactMaterialSMC()
    cone_mat.SetFriction(0.6)
    cone_mat.SetYoungModulus(2e5)
    for cone in scenario["cones"]:
        pts = c.vector_ChVector3d()
        for j in range(16):
            pts.push_back(
                c.ChVector3d(
                    0.25 * math.cos(j * math.pi / 8), 0.25 * math.sin(j * math.pi / 8), -0.2
                )
            )
        pts.push_back(c.ChVector3d(0, 0, 0.5))
        body = c.ChBodyEasyConvexHull(pts, 100, show_visual, True, cone_mat)
        body.SetMass(3)
        body.SetPos(c.ChVector3d(cone["x"], cone["y"], 0.205))
        body.SetFixed(True)
        system.AddBody(body)
        ids.add(body.GetIdentifier())
        cone_bodies.append(body)

    class Hits(c.ReportContactCallback):
        def __init__(self):
            super().__init__()
            self.hit = False

        def OnReportContact(self, pa, pb, plane, gap, radius, force, torque, a, b, offset):
            pair = {a.GetPhysicsItem().GetIdentifier(), b.GetPhysicsItem().GetIdentifier()}
            if ground.GetGroundBody().GetIdentifier() not in pair and pair & ids:
                self.hit = True
            return True

    hits = Hits()

    def measure():
        body = car.GetChassisBody()
        q = body.GetRot()
        f = q.Rotate(c.ChVector3d(1, 0, 0))
        vel = q.RotateBack(body.GetLinVel())
        pos = car.GetVehicle().GetPos()
        delta = 0.5 * (
            car.GetVehicle().GetSteeringAngle(0, v.LEFT)
            + car.GetVehicle().GetSteeringAngle(0, v.RIGHT)
        )
        return [pos.x, pos.y, math.atan2(f.y, f.x), vel.x, vel.y, body.GetAngVelLocal().z, delta]

    visualizer = None

    def advance(u):
        inp = v.DriverInputs()
        inp.m_steering, inp.m_throttle, inp.m_braking = u
        t = system.GetChTime()
        h = cfg["physics_dt_s"]
        terrain.Synchronize(t)
        car.Synchronize(t, inp, terrain)
        if visualizer:
            visualizer.synchronize(t, inp)
        terrain.Advance(h)
        car.Advance(h)
        if visualizer:
            visualizer.advance(h)
        system.GetContactContainer().ReportAllContacts(hits)

    for _ in range(round(cfg["settling_s"] / cfg["physics_dt_s"])):
        advance([0.0, 0.0, 0.3])
    start = system.GetChTime()
    s = measure()
    initial = s.copy()
    if mode == "easy":
        controller = EasyTracker(plan_nodes)
    else:
        controller_path = Path(controller_path).resolve()
        sys.path.insert(0, str(controller_path.parent))
        spec = importlib.util.spec_from_file_location("student_controller", controller_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        controller = module.Controller()
    controller.reset(copy.deepcopy(scenario), seed)
    if show_visual:
        from visual_course import CourseVisual

        visualizer = CourseVisual(car, ground, cone_bodies, scenario)
    previous = 0.0
    applied = [0.0, 0.0, 0.3]
    gate = 0
    rows = []
    requested_actions = []
    latencies = []
    reason = None
    finish_time = None
    minimum = 100.0
    error = None
    crossings = []
    for k in range(round(cfg["timeout_s"] / cfg["control_dt_s"])):
        if visualizer and not visualizer.draw(k * cfg["control_dt_s"], gate):
            reason = "window_closed"
            break
        obs = {
            "schema_version": cfg["schema_version"],
            "time": k * cfg["control_dt_s"],
            "dt": cfg["control_dt_s"],
            "state": s.copy(),
            "next_gate": gate,
            "previous_applied": dict(zip(["steering", "throttle", "brake"], applied)),
            "remaining_time": cfg["timeout_s"] - k * cfg["control_dt_s"],
        }
        before = time.perf_counter()
        try:
            command = controller.act(obs)
            applied = (
                apply_easy_action(command, previous, cfg)
                if mode == "easy"
                else apply_action(command, previous, cfg, s[3])
            )
        except Exception as exc:
            reason = "controller_error"
            error = f"{type(exc).__name__}: {exc}"
            break
        latencies.append(time.perf_counter() - before)
        previous = applied[0]
        for _ in range(round(cfg["control_dt_s"] / cfg["physics_dt_s"])):
            old = s
            old_time = system.GetChTime() - start
            advance(applied)
            s = measure()
            d = min(footprint_clearance(s, cone, cfg) for cone in scenario["cones"])
            minimum = min(minimum, d)
            if hits.hit or d <= 0:
                reason = "cone_contact"
            elif not inside_road(s, cfg):
                reason = "road_departure"
            elif abs(s[2]) > cfg["max_abs_heading_rad"]:
                reason = "heading_limit"
            elif (
                s[3] > cfg["max_forward_speed_m_s"]
                or (s[0] - old[0]) / cfg["physics_dt_s"] > cfg["max_forward_speed_m_s"]
            ):
                reason = "speed_limit"
            elif s[3] < cfg["min_forward_speed_m_s"]:
                reason = "reverse_limit"
            if gate < len(scenario["cones"]):
                cone = scenario["cones"][gate]
                if old[0] < cone["x"] <= s[0]:
                    fraction = (cone["x"] - old[0]) / (s[0] - old[0])
                    yy = old[1] + fraction * (s[1] - old[1])
                    if not gate_side_valid(yy, cone):
                        reason = reason or "wrong_gate_side"
                    else:
                        crossings.append(
                            {
                                "gate_index": gate,
                                "time_s": old_time + fraction * cfg["physics_dt_s"],
                                "Y_ref_m": yy,
                            }
                        )
                        gate += 1
            if reason:
                break
            if gate == len(scenario["cones"]) and old[0] < cfg["finish_x_m"] <= s[0]:
                fraction = (cfg["finish_x_m"] - old[0]) / (s[0] - old[0])
                finish_time = old_time + fraction * cfg["physics_dt_s"]
                reason = "success"
                break
        rows.append([*s, *applied, system.GetChTime() - start])
        if mode == "full":
            requested_actions.append([command["steering"], command["acceleration"]])
        if reason:
            break
    result = {
        "schema_version": cfg["schema_version"],
        "scenario_id": cfg["scenario_id"],
        "seed": seed,
        "mode": mode,
        "planned_stages": planned_stages,
        "status": reason or "timeout",
        "finish_time_s": finish_time,
        "elapsed_s": system.GetChTime() - start,
        "gates_passed": gate,
        "min_clearance_m": minimum,
        "initial_state": initial,
        "final_state": s,
        "median_action_ms": float(np.median(latencies) * 1000) if latencies else None,
        "p95_action_ms": float(np.quantile(latencies, 0.95) * 1000) if latencies else None,
        "error": error,
        "timing_enforced": False,
    }
    result["gate_crossings"] = crossings
    result["last_cone_time_s"] = (
        crossings[-1]["time_s"] if len(crossings) == len(scenario["cones"]) else None
    )
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    trajectory = {"rows": np.array(rows)}
    if mode == "full":
        trajectory["requested_actions"] = np.asarray(requested_actions, dtype=float).reshape(-1, 2)
    np.savez_compressed(output / "trajectory.npz", **trajectory)
    if visualizer:
        visualizer.show_result(result["status"], finish_time)
        visualizer.close()
    print(json.dumps(result, indent=2))
    return result


if __name__ == "__main__":
    root = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["easy", "full"], default="full")
    ap.add_argument("--plan", help="Easy-mode JSON move list")
    ap.add_argument("--controller", default=str(root / "controller.py"))
    ap.add_argument("--config", default=str(root / "course.json"))
    ap.add_argument("--output", default=str(root / "runs/local"))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument(
        "--visual",
        action="store_true",
        help="Show a live PyChrono window with a marked finish line",
    )
    args = ap.parse_args()
    result = run(
        args.controller,
        args.config,
        args.output,
        args.seed,
        mode=args.mode,
        plan_path=args.plan,
        show_visual=args.visual,
    )
    raise SystemExit(0 if result["status"] == "success" else 1)
