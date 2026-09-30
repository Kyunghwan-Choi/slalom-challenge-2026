# From the example run to your own method

Complete [installation](INSTALL.md), then work from the repository root in Miniconda Prompt. The supplied controller is a slow interface check, not a learning-based solution.

## 1. Run and inspect

```bat
conda run --no-capture-output -n slalom2026 python run_local.py --mode full --controller controller.py --output runs\starter
```

Save this inspection snippet as `inspect_run.py` in the repository root, then run `conda run --no-capture-output -n slalom2026 python inspect_run.py`:

```python
import json
from pathlib import Path
import numpy as np

folder = Path("runs/starter")
result = json.loads((folder / "result.json").read_text(encoding="utf-8"))
with np.load(folder / "trajectory.npz") as trace:
    rows = trace["rows"]
    requests = trace["requested_actions"]

print(result["status"], result["gates_passed"], result["finish_time_s"])
if len(rows):
    print("before first action:", result["initial_state"])
    print("first request:", requests[0])
    print("after first action:", rows[0, :7])
    print("applied steering, throttle, brake:", rows[0, 7:10])
```

For row `k`, the state **before** `requested_actions[k]` is `initial_state` when `k=0`, or `rows[k-1, :7]` otherwise. `rows[k, :7]` is the state afterward; `rows[k, 10]` is elapsed simulation time. A terminal event can interrupt a 0.02 s step. Use `result.json` for finish and gate times. If the run stops before creating output files, inspect the terminal error.

## 2. Try one model prediction

In a Full-mode `Controller.act`, compare any valid candidate action with the supplied one-step predictor:

```python
from dynamics import predict

candidate = {"steering": 0.0, "acceleration": 0.0}  # interface example
next_state, applied_inputs = predict(
    observation["state"],
    candidate,
    observation["previous_applied"]["steering"],
    self.scenario,
)
```

`next_state` has seven entries; `applied_inputs` is `[steering, throttle, brake]`. The predictor includes the public steering rate limit and acceleration-to-pedal adapter. A zero acceleration request can still apply throttle to offset modeled resistance. The predictor approximates PyChrono, so compare predictions with recorded outcomes. See [Interface](INTERFACE.md) and [Model](MODEL.md) for units, bounds, and limits.

## 3. Connect your learned artifact

Collect your own runs, train your chosen method offline, and save its required arrays, for example in `learned.npz` beside your submitted `controller.py`. The following is **illustrative wiring**: create the artifact and implement `select_action` using your own method before running it.

```python
from pathlib import Path
import numpy as np

class Controller:
    def reset(self, scenario, seed):
        self.scenario = scenario
        path = Path(__file__).resolve().parent / "learned.npz"
        with np.load(path, allow_pickle=False) as saved:
            self.learned = {name: saved[name] for name in saved.files}

    def act(self, observation):
        steering, acceleration = self.select_action(observation, self.learned)
        return {"steering": float(steering), "acceleration": float(acceleration)}
```

Test the finished controller with `conda run --no-capture-output -n slalom2026 python run_local.py --mode full --controller your_controller.py --output runs/your_run`. Submit the artifact and code needed to reproduce it; evaluation loads your files without retraining or installing extra packages. Easy mode uses a complete `easy_plan.json` instead; see [Running](RUNNING.md) and [Submission](SUBMISSION_AND_AI.md).
