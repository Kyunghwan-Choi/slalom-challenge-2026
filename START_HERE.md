# From the 5 m/s example controller to your own method

Complete [installation](INSTALL.md), then work from the repository root in Miniconda Prompt. The **5 m/s example controller** demonstrates the interface; develop your own course-based method.

## 1. Run and inspect

```bat
conda run --no-capture-output -n slalom2026 python run_local.py --mode full --controller controller.py --output runs\example_controller
```

Save this inspection snippet as `inspect_run.py` in the repository root, then run `conda run --no-capture-output -n slalom2026 python inspect_run.py`:

```python
import json
from pathlib import Path
import numpy as np

folder = Path("runs/example_controller")
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

For row `k`, the state **before** `requested_actions[k]` is `initial_state` when $k=0$, or `rows[k-1, :7]` otherwise. `rows[k, :7]` is the state afterward; `rows[k, 10]` is elapsed simulation time. A terminal event can interrupt a 0.02 s step. Use `result.json` for finish and gate times. If the run stops before creating output files, inspect the terminal error.

## 2. Predict and compare

After the run above, execute [try_predict.py](try_predict.py) to predict one recorded action and print the predicted state, measured state, and their difference. Row 500 starts at 10 s:

```bat
conda run --no-capture-output -n slalom2026 python try_predict.py --run runs\example_controller --row 500
```

Add `--steps 25` to replay the next 0.5 s of recorded commands from the same initial measurement. The script carries the predicted state and previous applied steering forward; it does not reset to measured states between steps. This demonstrates model use, not action optimization. In your controller, choose candidate actions with your own method; [Interface](INTERFACE.md#action-and-input-adapter) shows the `predict` call.

`predict` returns seven state entries and `[steering, throttle, brake]`. It includes the input adapter; a zero acceleration request may still apply throttle to offset modeled resistance. The script uses the supplied basic course and rejects partial terminal steps. [Model](MODEL.md) explains prediction errors; [failure diagnosis](INTERFACE.md#diagnose-a-failed-drive) explains driving outcomes.

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

For a basic-course Full-mode run, `conda run --no-capture-output -n slalom2026 python validate_requests.py --run runs/your_run` writes `model_check.json` beside the driving results. It compares the supplied model with your recorded commands at available 0.5, 1, and 2 s prediction horizons; at least 0.5 s of complete control steps is required.
