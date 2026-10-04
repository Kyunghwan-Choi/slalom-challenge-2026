"""Check the supplied predictor against logged high-level acceleration requests.

The plots need Matplotlib; the metric calculation only needs the course Conda lock.
The traces include a separate faster slalom recorded after the model fit.
"""

import argparse
import json
from pathlib import Path

import numpy as np

from dynamics import predict

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "validation"
CONFIG = json.loads((ROOT / "course.json").read_text(encoding="utf-8"))
CASES = {
    "starter_slalom": "request_starter.npz",
    "faster_slalom": "request_fast_slalom.npz",
    "high_speed_straight": "request_straight.npz",
}
LABELS = {
    "starter_slalom": "5 m/s example controller",
    "faster_slalom": "8 m/s slalom: separate validation drive",
    "high_speed_straight": "Straight speed probe: calibration data",
}


def metrics_for_arrays(states, requests, applied):
    if states.shape != (len(requests) + 1, 7) or applied.shape != (len(requests), 3):
        raise ValueError(
            "Expected one initial state, seven-state samples, and applied input triples"
        )
    if requests.shape != (len(applied), 2):
        raise ValueError("Expected steering and acceleration requests from a Full-mode run")
    dt = CONFIG["control_dt_s"]
    metrics = {}
    plot_data = None
    for horizon in (0.5, 1.0, 2.0):
        steps = round(horizon / dt)
        if len(requests) < steps:
            continue
        predicted = []
        observed = []
        endpoints = []
        for start in range(0, len(requests) - steps + 1, round(0.5 / dt)):
            state = states[start].copy()
            previous_steering = float(applied[start - 1, 0]) if start else 0.0
            for k in range(start, start + steps):
                command = {
                    "steering": float(requests[k, 0]),
                    "acceleration": float(requests[k, 1]),
                }
                state, model_input = predict(state, command, previous_steering, CONFIG)
                previous_steering = float(model_input[0])
            predicted.append(state)
            observed.append(states[start + steps])
            endpoints.append((start + steps) * dt)
        predicted = np.asarray(predicted)
        observed = np.asarray(observed)
        error = predicted - observed
        metrics[f"{horizon:g}_s"] = {
            "windows": len(error),
            "vx_endpoint_rmse_m_s": float(np.sqrt(np.mean(error[:, 3] ** 2))),
            "y_ref_endpoint_rmse_m": float(np.sqrt(np.mean(error[:, 1] ** 2))),
        }
        if horizon == 2.0:
            plot_data = (np.asarray(endpoints), observed, predicted)
    if not metrics:
        raise ValueError("A model check needs at least 0.5 s of recorded Full-mode commands")
    return {
        "speed_range_m_s": [float(np.min(states[:, 3])), float(np.max(states[:, 3]))],
        "request_range_m_s2": [float(np.min(requests[:, 1])), float(np.max(requests[:, 1]))],
        "metrics": metrics,
    }, plot_data


def case_metrics(filename):
    with np.load(OUT / filename) as data:
        report, plot_data = metrics_for_arrays(data["states"], data["requests"], data["applied"])
    return {"trace": filename, **report}, plot_data


def check_run(folder, output):
    folder = Path(folder)
    result = json.loads((folder / "result.json").read_text(encoding="utf-8"))
    if result.get("mode") != "full" or result.get("scenario_id") != CONFIG["scenario_id"]:
        raise ValueError("Use a Full-mode run on the supplied basic course for this model check")
    with np.load(folder / "trajectory.npz") as trace:
        rows = trace["rows"]
        requests = trace["requested_actions"]
    if len(rows) and rows[-1, 10] < len(rows) * CONFIG["control_dt_s"] - 1e-6:
        rows, requests = rows[:-1], requests[:-1]
    states = np.vstack((result["initial_state"], rows[:, :7]))
    report, _ = metrics_for_arrays(states, requests, rows[:, 7:10])
    report = {
        "scenario_id": result["scenario_id"],
        "drive_status": result["status"],
        "finish_time_s": result["finish_time_s"],
        **report,
    }
    output = Path(output) if output else folder / "model_check.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return report


def plot_responses(series):
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("Matplotlib unavailable; metrics were written without regenerating the plot.")
        return
    fig, axes = plt.subplots(
        len(series), 2, figsize=(11.8, 3.1 * len(series)), constrained_layout=True, squeeze=False
    )
    for row, (name, data) in enumerate(series.items()):
        time, observed, predicted = data
        for column, (index, ylabel) in enumerate(
            ((3, "COM forward speed (m/s)"), (1, "REF lateral position (m)"))
        ):
            ax = axes[row, column]
            ax.plot(time, observed[:, index], color="#172554", linewidth=2, label="PyChrono")
            ax.plot(
                time,
                predicted[:, index],
                color="#ea580c",
                linewidth=1.8,
                linestyle="--",
                label="Control-oriented vehicle model",
            )
            ax.set_title(LABELS[name])
            ax.set_xlabel("2 s window endpoint (s)")
            ax.set_ylabel(ylabel)
            ax.grid(alpha=0.22)
            ax.legend(frameon=False)
    fig.suptitle("Response to recorded steering and acceleration requests")
    fig.savefig(OUT / "ax_request_comparison.png", dpi=160)
    plt.close(fig)


def main():
    report = {
        "method": "2 s/1 s/0.5 s open-loop prediction windows restarted from measured states every 0.5 s; logged high-level requests replayed through dynamics.predict",
        "scope": "Friction 0.9; 5 m/s example controller, separate 8 m/s slalom recorded with frozen model parameters, and a calibration-related straight speed probe. Errors describe measured-state prediction windows at the listed horizons.",
        "cases": {},
    }
    plots = {}
    for name, filename in CASES.items():
        report["cases"][name], plots[name] = case_metrics(filename)
    (OUT / "ax_request_metrics.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    plot_responses(plots)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--run", help="Check result.json and trajectory.npz from your basic-course Full run"
    )
    parser.add_argument(
        "--output", help="Metric JSON path; with --run, defaults to <run>/model_check.json"
    )
    args = parser.parse_args()
    if args.run:
        check_run(args.run, args.output)
    else:
        if args.output:
            parser.error("--output requires --run")
        main()
