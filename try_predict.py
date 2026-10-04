"""Compare model predictions with a recorded Full-mode run; no controller optimization."""

import argparse
import json
from pathlib import Path

import numpy as np

from dynamics import predict


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, default=Path("runs/example_controller"))
    parser.add_argument("--row", type=int, default=0, help="Zero-based action row")
    parser.add_argument("--steps", type=int, default=1, help="Number of recorded actions to replay")
    args = parser.parse_args()
    if args.row < 0 or args.steps < 1:
        parser.error("--row must be nonnegative and --steps must be positive")

    config = json.loads(Path(__file__).with_name("course.json").read_text(encoding="utf-8"))
    result = json.loads((args.run / "result.json").read_text(encoding="utf-8"))
    if result["mode"] != "full" or result["scenario_id"] != config["scenario_id"]:
        parser.error("Use a Full-mode run on the supplied basic course")
    with np.load(args.run / "trajectory.npz", allow_pickle=False) as trace:
        rows = trace["rows"]
        requests = trace["requested_actions"]

    end = args.row + args.steps
    if end > len(rows):
        parser.error("The selected action range exceeds the recorded run")
    durations = np.diff(np.r_[0.0, rows[:, 10]])
    dt = config["control_dt_s"]
    if not np.allclose(durations[args.row : end], dt, rtol=0, atol=1e-6):
        parser.error("Select complete control steps; a terminal event may shorten the final step")

    # Row k stores the state AFTER request k; the initial measurement precedes row 0.
    state = np.array(result["initial_state"] if args.row == 0 else rows[args.row - 1, :7])
    previous_steering = 0.0 if args.row == 0 else float(rows[args.row - 1, 7])
    for k in range(args.row, end):
        command = {"steering": float(requests[k, 0]), "acceleration": float(requests[k, 1])}
        state, applied_inputs = predict(state, command, previous_steering, config)
        previous_steering = float(applied_inputs[0])

    measured = rows[end - 1, :7]
    print(f"Start row {args.row}; prediction horizon {args.steps * dt:g} s")
    print("First recorded request [steering, acceleration]:", requests[args.row])
    print(f"{'State (unit)':<20} {'Predicted':>12} {'Measured':>12} {'Difference':>12}")
    labels = (
        "X_REF (m)",
        "Y_REF (m)",
        "psi (rad)",
        "vx (m/s)",
        "vy (m/s)",
        "r (rad/s)",
        "delta (rad)",
    )
    for label, predicted, observed in zip(labels, state, measured):
        print(f"{label:<20} {predicted:12.6f} {observed:12.6f} {predicted - observed:12.6f}")


if __name__ == "__main__":
    main()
