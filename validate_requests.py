"""Check the supplied predictor against logged high-level acceleration requests.

The plots need Matplotlib; the metric calculation only needs the course Conda lock.
These replay tests do not implement a controller or establish race-time accuracy.
"""

from pathlib import Path
import json

import numpy as np

from dynamics import predict


ROOT = Path(__file__).resolve().parent
OUT = ROOT / "validation"
CONFIG = json.loads((ROOT / "course.json").read_text(encoding="utf-8"))
CASES = {
    "starter_slalom": "request_starter.npz",
    "high_speed_straight": "request_straight.npz",
}


def case_metrics(filename):
    with np.load(OUT / filename) as data:
        states = data["states"]
        requests = data["requests"]
        applied = data["applied"]
    assert states.shape == (len(requests) + 1, 7)
    assert applied.shape == (len(requests), 3)
    dt = CONFIG["control_dt_s"]
    metrics = {}
    plot_data = None
    for horizon in (0.5, 1.0, 2.0):
        steps = round(horizon / dt)
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
            plot_data = (np.asarray(endpoints), observed[:, 3], predicted[:, 3])
    return {
        "trace": filename,
        "speed_range_m_s": [float(np.min(states[:, 3])), float(np.max(states[:, 3]))],
        "request_range_m_s2": [float(np.min(requests[:, 1])), float(np.max(requests[:, 1]))],
        "metrics": metrics,
    }, plot_data


def plot_responses(series):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("Matplotlib unavailable; metrics were written without regenerating the plot.")
        return
    fig, axes = plt.subplots(1, 2, figsize=(11.8, 3.9), constrained_layout=True)
    for ax, (name, data) in zip(axes, series.items()):
        time, observed, predicted = data
        ax.plot(time, observed, color="#172554", linewidth=2, label="PyChrono")
        ax.plot(time, predicted, color="#ea580c", linewidth=1.8,
                linestyle="--", label="Reduced model")
        ax.set_title("Supplied 5 m/s slalom" if name == "starter_slalom"
                     else "High-speed straight probe")
        ax.set_xlabel("2 s window endpoint (s)")
        ax.set_ylabel("COM forward speed (m/s)")
        ax.grid(alpha=0.22)
        ax.legend(frameon=False)
    fig.suptitle("Forward-speed response to logged steering and acceleration requests")
    fig.savefig(OUT / "ax_request_comparison.png", dpi=160)
    plt.close(fig)


def main():
    report = {
        "method": "2 s/1 s/0.5 s open-loop prediction windows restarted from measured states every 0.5 s; logged high-level requests replayed through dynamics.predict",
        "limits": "Starter slalom and a calibration-related straight probe; neither establishes handling-limit or full-course open-loop accuracy.",
        "cases": {},
    }
    plots = {}
    for name, filename in CASES.items():
        report["cases"][name], plots[name] = case_metrics(filename)
    (OUT / "ax_request_metrics.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    plot_responses(plots)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
