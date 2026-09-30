# Slalom Challenge

This repository contains the student environment for the 2026 **Learning-Based Control for Mobility Systems** slalom challenge. Drive a BMW E90 through eight alternating cone gates and reach the finish as quickly as possible. A valid run must respect the course and vehicle limits and avoid cone contact. Your control design should connect to the methods studied in class.

You can work in either of two modes on the same PyChrono course:

- **Easy:** plan a sequence of grid moves in a fixed-time virtual motion model. A supplied lower-level controller tracks your plan in PyChrono.
- **Full:** implement a controller that chooses a steering request and desired longitudinal acceleration from vehicle observations. You may also add your own lower-level tracking controller and apply learning-based methods to a higher-level planner. A supplied open-loop acceleration-to-pedal adapter and control-oriented model are available for design and prediction.

The Full-mode acceleration interface does not change Easy mode's plan format, shared tracker, or native pedal commands. Both modes run on the same PyChrono course and use the same success rules.

The repository includes the course configuration, local simulator, controller example, model, validation data, and interface checks. The example controller follows a low-speed path so that you can verify the installation and understand the input/output contract.

## Documentation

| Read | What it covers |
|---|---|
| [Assignment](ASSIGNMENT.md) | Objective, course, success and failure conditions, schedule, and evaluation principles |
| [Installation](INSTALL.md) | Windows x64 setup and environment checks |
| [Running and interfaces](RUNNING.md) | Example run, Easy and Full modes, inputs, outputs, and model evidence |
| [Submission and AI use](SUBMISSION_AND_AI.md) | Required files, report template, AI disclosure, and discussion session |
| [Troubleshooting](TROUBLESHOOTING.md) | How to report an installation or simulator issue |
| [Questions and answers](Q_AND_A.md) | Common questions about the two modes, actions, and evaluation |

Install the environment first, then run the supplied controller and choose a mode in [Running and interfaces](RUNNING.md). The local runner writes `result.json` and `trajectory.npz` to the output directory you specify.
