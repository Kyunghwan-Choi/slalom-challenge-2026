# Slalom Challenge

In this individual 2026 **Learning-Based Control for Mobility Systems** challenge, drive a PyChrono BMW E90 through eight alternating cones and finish as fast as possible. Stay on the road, avoid contact, respect vehicle limits, and justify your design using course methods.

Choose one mode; both use the same course and success rules.

| Mode | Your task | Supplied support |
|---|---|---|
| **Easy** | Submit grid moves in `easy_plan.json` | Virtual model and fixed plan tracker |
| **Full** | Submit a `Controller` requesting steering and acceleration every 0.02 s | Vehicle predictor, pedal adapter, and slow example |

Full mode may combine a learned planner with your own tracker; `act` still returns the published action. Easy mode keeps its separate plan and fixed tracker.

The slow example checks installation and the interface; its path and speed are not assigned references. Course configuration, runner, model validation data, and interface checks are included.

## Start

1. [Install](INSTALL.md) the environment.
2. [Run the slow example](RUNNING.md) and inspect `result.json` and `trajectory.npz`.
3. Choose a mode, implement your method, and test it with the local runner.

## Guides

| Guide | Contents |
|---|---|
| [Assignment](ASSIGNMENT.md) | Rules, dates, scoring |
| [Installation](INSTALL.md) | Windows x64 setup |
| [Running](RUNNING.md) | Both modes and local tests |
| [Interface](INTERFACE.md) | Inputs and outputs |
| [Model](MODEL.md) | Predictor and validation |
| [Submission and AI](SUBMISSION_AND_AI.md) | Files, report, AI disclosure |
| [Troubleshooting](TROUBLESHOOTING.md) | Technical issues |
| [Q&A](Q_AND_A.md) | Common questions |
