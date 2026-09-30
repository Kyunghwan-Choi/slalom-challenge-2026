# Run and develop a controller

Complete [installation](INSTALL.md) first. Run commands from the repository root in PowerShell or Miniconda Prompt. Both modes use the same BMW E90/TMeasy PyChrono course and success rules; `--mode` selects what you submit.

## Check the installation

```powershell
conda run --no-capture-output -n slalom2026 python .\run_local.py --mode full --controller .\controller.py --output .\runs\starter
```

The supplied [`controller.py`](controller.py) tracks its own sinusoidal path at **5 m/s** with a proportional acceleration request. It demonstrates the interface; its path and speed are not assigned references or a qualifying learning-based solution. In one headless PyChrono 10.0.0 run it passed all eight gates in **30.660 s**. Check `runs/starter/result.json` for `"status": "success"`, `gates_passed: 8`, and `finish_time_s` (simulation time); small numerical differences are possible.

Runs are headless by default. Add `--visual` in a Windows graphical session to inspect the car and finish line; use headless runs for reported times. A completed runner call writes `result.json` and `trajectory.npz`. A malformed plan, controller import/reset error, or simulator exception can stop before either file is written; check the terminal error. See [INTERFACE.md](INTERFACE.md) for output fields.

## Choose a mode

| | Easy: grid plan | Full: vehicle control |
|---|---|---|
| Submit | Integer moves in `easy_plan.json` | `Controller.reset` and `Controller.act` in a Python file |
| Interval | 0.5 s virtual plan; supplied tracker acts every 0.02 s | Your controller acts every 0.02 s |
| Supplied | Exact virtual transitions and fixed tracker | Identified seven-state predictor and acceleration-to-pedal adapter |
| Evaluated in | PyChrono | PyChrono |

### Easy: finite-horizon DP

The planning state is $(i,j,m_{\mathrm{prev}},n_{\mathrm{prev}})$, initially $(0,0,0,0)$. Planned REF position is $(X,Y)=(i h_X,j h_Y)$ with $h_X=1$ m and $h_Y=0.5$ m. Every 0.5 s, choose integers $m\in\{1,2,3,4,5\}$ and $n\in\{-4,\ldots,4\}$:

$$
i^+=i+m,\qquad j^+=j+n,\qquad
V=\frac{\sqrt{(m h_X)^2+(n h_Y)^2}}{0.5},\qquad
\chi=\operatorname{atan2}(n h_Y,m h_X).
$$

Stored previous moves determine acceleration and turn limits. $V$ and $\chi$ are the virtual REF segment speed and global course angle, not exact COM speed or body yaw. The grid assumes each move is achieved exactly; PyChrono tests the supplied tracker's actual motion. [`easy_spec.json`](easy_spec.json) defines these limits:

| Planning limit | Value |
|---|---:|
| Lateral position $|Y|$ | $\le3$ m |
| Speed $V$ | $\le11.5$ m/s |
| Course angle $|\chi|$ | $\le0.58$ rad |
| Acceleration / deceleration | $\le4$ / $\le6$ m/s² |
| Angle change $|\Delta\chi|$ | $\le0.31$ rad |
| Virtual lateral acceleration $V|\Delta\chi|/0.5$ | $\le6$ m/s² |

At each cone's X plane, the planned segment must cross on the designated side. The grid omits the vehicle footprint: choose a clearance margin and verify it in PyChrono.

The first arrival at $X\ge145$ m must be by stage $K\le120$. Each feasible move costs **1**, including the finish move; a successful terminal state adds **0**. An invalid move or unfinished horizon incurs $M=1000$. Success therefore costs exactly $K$; failure costs at least $M$. All moves increase X, so backward DP is possible. Other numerical methods are allowed if you explain yours. The repository supplies the model, checker, and tracker, but no solver or optimal move list.

For the virtual side-only gate rule, backward DP found a **31-stage** path (15.5 s planned), with only 0.5 m planned lateral cone separation. That alone does not establish PyChrono safety. A separate plan with a **2.5 m planning margin** took **48 stages** (24.0 s planned); its tracker passed all eight gates in PyChrono in **24.074 s**, and its saved trajectory clears the published COM-centered scoring footprint. The margin is a planning choice, not an evaluation rule. Neither result establishes real-vehicle optimality: tracking error and omitted tire/actuator states can make grid-feasible plans fail.

Write the full move list as JSON:

```json
{"moves": [[1, 0], [2, 0]]}
```

These two feasible initial moves only show the format; the runner rejects them as unfinished. Each pair must contain integers, and the last move must be the **first** to reach or cross the finish. Run your complete plan:

```powershell
conda run --no-capture-output -n slalom2026 python .\run_local.py --mode easy --plan .\easy_plan.json --output .\runs\easy
```

The validator checks the grid, transitions, and planned gates. [`easy_mode.py`](easy_mode.py) converts accepted nodes to steering and pedals using the same tracker for everyone. The Full-mode acceleration request interface does not change it. Only PyChrono determines actual success, collision, and finish time.

### Full: vehicle controller

The supplied predictor uses

$$
s=(X_{\mathrm{REF}},Y_{\mathrm{REF}},\psi,v_x,v_y,r,\delta),
\qquad \hat s_{k+1}=F_{\mathrm{red}}(s_k,u_k,u_{s,k-1}^{\mathrm{applied}};\Delta t=0.02\,{\rm s},\mu=0.9).
$$

$v_x,v_y$ are body-frame COM velocities; $r$ is yaw rate; $\delta$ is measured mean front-wheel steering. The action $u_k=(u_{s,k}^{\rm req},a_{x,k}^{\rm req})$ requests steering and longitudinal acceleration. Prediction needs the previously applied steering request for the rate limit. The open-loop adapter converts acceleration to throttle or brake, with speed-dependent limits. [`MODEL.md`](MODEL.md) explains the equations, coordinates, evidence, and validity range; [`dynamics.py`](dynamics.py) and [`model_parameters.json`](model_parameters.json) implement the fitted predictor. It is not an exact PyChrono model.

Fixed **native-input** predictions restart at measured states for each validation window. At 2 s, the endpoint RMSEs were:

| Maneuver | Forward speed | REF lateral position |
|---|---:|---:|
| Held-out mixed | 0.0832 m/s | 0.0382 m |
| Baseline course, included in fitting | 0.2401 m/s | 0.0560 m |
| Separate held-out aggressive slalom | 0.4439 m/s | 0.2111 m |

The [plot](validation/model_comparison.png) and [`metrics.json`](validation/metrics.json) document the public traces; [MODEL.md](MODEL.md) explains the separate aggressive result. These tests use applied steering and pedals, so they do not measure acceleration-request tracking. Short windows do not establish full-course open-loop accuracy or handling-limit reliability. Run `validate_model.py` to reproduce the public-trace plot and metrics.

Copy [`controller.py`](controller.py) and implement `Controller.reset` and `Controller.act`. Every 0.02 s, `act` receives an observation and returns steering and acceleration requests. Acceleration may be requested in $[-7,7]$ m/s², but the adapter clips it to the speed-dependent achievable range. [`dynamics.predict`](dynamics.py) includes that conversion for candidate-action evaluation. [INTERFACE.md](INTERFACE.md) defines the lifecycle, units, bounds, and scenario fields.

You may collect trajectories, improve the model, fit a value or policy approximation, or combine offline training with online improvement. You may also put a learning-based planner and your own tracker inside the same `Controller`; `act` must still return `{"steering", "acceleration"}`. Preserve the published evaluation interface. The base environment lacks optional Numba acceleration; measure runtime when evaluating many candidates. [Start here](START_HERE.md) shows a first predictor call, run-data check, and way to load a learned artifact.

```powershell
conda run --no-capture-output -n slalom2026 python .\run_local.py --mode full --controller .\my_controller.py --output .\runs\full
```

For reproducible comparisons, record the scenario, seed, mode, outcome, finish time, and controller version. `--config` selects a course JSON; the basic scored course is [`course.json`](course.json). For local spacing tests, copy it, set a new `scenario_id`, change `cone_spacing_m`, and place `finish_x_m` beyond the last cone. Keep the JSON with the result because `result.json` records its `scenario_id`, not every course parameter. Local experiments may instead specify `cone_centers_m` as eight `[x,y]` pairs in increasing X order (Y may vary); this overrides `cone_spacing_m`, and pass sides still alternate.

The official **Full-mode-only** bonus changes cone spacing only; its range and scoring will be announced separately. Use the same controller and learned object for basic and bonus runs. That unchanged controller or learned object may read the spacing value.
