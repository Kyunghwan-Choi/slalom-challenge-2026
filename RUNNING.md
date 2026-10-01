# Run and develop a controller

Complete [installation](INSTALL.md) first. Run commands from the repository root in PowerShell or Miniconda Prompt. Both modes use the same BMW E90/TMeasy PyChrono course and success rules; `--mode` selects the control task.

## Check the installation

```powershell
conda run --no-capture-output -n slalom2026 python .\run_local.py --mode full --controller .\controller.py --output .\runs\starter
```

The supplied [`controller.py`](controller.py) tracks a sample sinusoidal path at **5 m/s**. This low-speed drive checks installation and shows the controller interface; its path and speed are not assigned references or a learning-based solution. In one headless PyChrono 10.0.0 run it passed all eight gates in **30.660 s**. Check `runs/starter/result.json` for `"status": "success"`, `gates_passed: 8`, and `finish_time_s` (simulation time); small numerical differences are possible.

Runs are headless by default. Add `--visual` in a Windows graphical session to inspect the car and finish line; use headless runs for reported times. A completed runner call writes `result.json` and `trajectory.npz`. A malformed plan, controller import/reset error, or simulator exception can stop before either file is written; check the terminal error. See [INTERFACE.md](INTERFACE.md) for output fields.

## Choose a mode

| | Easy: waypoint plan | Full: vehicle control |
|---|---|---|
| Your task | Choose integer REF waypoint moves for virtual **0.5 s** stages in `easy_plan.json` | Write `Controller.reset` and `Controller.act` in Python; request steering and acceleration every **0.02 s** |
| Supplied support | Simplified grid model and fixed path-tracking controller, which acts every 0.02 s | Reduced control-oriented vehicle model and a low-speed example controller |
| Evaluated in | PyChrono | PyChrono |

### Easy: finite-horizon DP

The **planned REF position** at stage $k$ is $(X_k,Y_k)$, initially $(0,0)$, on a grid with $h_X=1$ m and $h_Y=0.5$ m. Every $\Delta=0.5$ s, choose integer move $(m_k,n_k)$:

$$
m_k\in\{1,2,3,4,5\},\quad n_k\in\{-4,\ldots,4\},
\qquad X_{k+1}=X_k+m_k h_X,\quad Y_{k+1}=Y_k+n_k h_Y.
$$

The move implies virtual segment speed and global course angle:

$$
V_k=\frac{\sqrt{(m_k h_X)^2+(n_k h_Y)^2}}{\Delta},\qquad
\chi_k=\operatorname{atan2}(n_k h_Y,m_k h_X).
$$

These are not exact COM speed or body yaw. The geometric planning state is $(X_k,Y_k)$, but acceleration and turn limits also need the **previous segment's** $(V_{k-1},\chi_{k-1})$. Carry these two values as auxiliary memory in a DP implementation; initialize both to zero. The grid assumes each move is achieved exactly. PyChrono later tests the actual car and the supplied tracker.

The planning constants come from [easy_spec.json](easy_spec.json); the finish position comes from [course.json](course.json):

| Symbol | Value | Meaning |
|---|---:|---|
| $h_X$, $h_Y$ | 1 m, 0.5 m | Grid spacing |
| $\Delta$ | 0.5 s | Planning stage duration |
| $K_{\max}$ | 120 | Last allowed arrival stage |
| $X_F$ | 145 m | Finish line |
| $M$ | 1000 | Failure cost |

A move is **admissible** only if every limit holds:

| Planning limit | Condition |
|---|---|
| Road strip | $\lvert Y_{k+1}\rvert\le3$ m |
| Segment speed | $V_k\le11.5$ m/s |
| Course angle | $\lvert\chi_k\rvert\le0.58$ rad |
| Acceleration | $(V_k-V_{k-1})/\Delta\le4$ m/s² |
| Deceleration | $(V_{k-1}-V_k)/\Delta\le6$ m/s² |
| Angle change | $\lvert\chi_k-\chi_{k-1}\rvert\le0.31$ rad |
| Virtual lateral acceleration | $V_k\lvert\chi_k-\chi_{k-1}\rvert/\Delta\le6$ m/s² |

At each cone X plane crossed by a planned segment, linearly interpolate the segment's Y coordinate. The crossing must be strictly on the cone's designated side: `pass_sign * (Y_cross - Y_cone) > 0`. This is a **virtual pass-side rule**, not a cone collision test. The grid omits the vehicle footprint; choose a planning clearance margin and verify the resulting path in PyChrono.

The first arrival at $X\ge X_F$ must occur by stage $K\le K_{\max}$. A stage cost consistent with this time-minimization task is

$$
g(z_k,m_k,n_k)=
\begin{cases}
1, & \text{admissible move, including the finish move},\\
M, & \text{limit or planned gate violation; terminate in failure},
\end{cases}
$$

where $z_k=(X_k,Y_k,V_{k-1},\chi_{k-1})$ includes the two constraint-memory values. On first reaching $X_F$, terminate with cost **0**; if stage $K_{\max}$ ends before the finish, assign terminal cost **$M$**. A successful plan therefore costs its stage count $K$; failure costs at least $M$. All moves increase X, so backward DP is possible. Other course-based numerical methods are allowed if explained. The repository supplies the model, checker, and tracker, but no solver or optimal move list. The runner rejects an invalid JSON plan instead of simulating its penalty; $M$ is for your optimization formulation.

For the virtual pass-side gate rule, backward DP found a **31-stage** path (15.5 s planned) with only **0.5 m planned lateral cone separation**. That path was **not verified as safe in PyChrono**. A separate plan with a **2.5 m planning margin** took **48 stages** (24.0 s planned); its tracker passed all eight gates in PyChrono in **24.074 s**, and its saved trajectory clears the published COM-centered scoring footprint. The margin is a planning choice, not an evaluation rule. Tracking error and omitted tire/actuator states can make grid-feasible plans fail.

Save **all** planned 0.5 s moves, from the starting position through the first finish-line crossing, in one JSON file. Its structure is:

```json
{"moves": [[1, 0], [2, 0]]}
```

These two initial moves only show the format; the runner rejects this example because it ends before the finish. Each pair contains integers `[m_k, n_k]`, and the last pair must be the **first** to reach or cross the finish. Run your complete plan:

```powershell
conda run --no-capture-output -n slalom2026 python .\run_local.py --mode easy --plan .\easy_plan.json --output .\runs\easy
```

The validator checks the grid, transitions, and planned gates. [easy_mode.py](easy_mode.py) converts accepted nodes to steering and pedals using the same tracker for everyone. The Full-mode acceleration request interface does not change it. Only PyChrono determines actual success, collision, and finish time.

### Full: vehicle controller

The supplied predictor uses

$$
s=(X_{\mathrm{REF}},Y_{\mathrm{REF}},\psi,v_x,v_y,r,\delta),
\qquad \hat s_{k+1}=F_{\mathrm{red}}(s_k,u_k,u_{s,k-1}^{\mathrm{applied}};\Delta t=0.02\,{\rm s},\mu=0.9).
$$

$v_x,v_y$ are body-frame COM velocities; $r$ is yaw rate; $\delta$ is measured mean front-wheel steering. The action $u_k=(u_{s,k}^{\rm req},a_{x,k}^{\rm req})$ requests steering and longitudinal acceleration. The runner limits the steering command sent to PyChrono to a change of **0.04 per 0.02 s**; the predictor includes that interface rule and a fitted steering lag, so prediction needs the previously applied steering. A fixed map converts acceleration requests to throttle or brake within speed-dependent limits. Use the model for model-based planning, policy improvement, or candidate-action prediction, then test the resulting controller in PyChrono. [MODEL.md](MODEL.md) explains the equations, coordinates, evidence, and validity range; [dynamics.py](dynamics.py) and [model_parameters.json](model_parameters.json) implement the fitted predictor.

**Requested-action validation** replays recorded Full-mode steering and acceleration requests through `dynamics.predict`. Each 2 s window starts from a measured PyChrono state; the reported errors are at the window endpoint.

| PyChrono drive | Windows | Forward-speed RMSE | REF lateral-position RMSE |
|---|---:|---:|---:|
| Low-speed starter slalom | 58 | 0.3453 m/s | 0.0983 m |
| Straight drive to 10.74 m/s, calibration-related | 10 | 0.1184 m/s | 0.0777 m |

The straight drive is **not** an independent high-speed slalom test. These errors include the complete response to your requested actions; they do not guarantee full-course open-loop accuracy or behavior near handling limits. See the [comparison plot](validation/ax_request_comparison.png), [metrics](validation/ax_request_metrics.json), and [MODEL.md](MODEL.md) for conditions and limitations.

Copy [`controller.py`](controller.py) and implement `Controller.reset` and `Controller.act`. Every 0.02 s, `act` receives an observation and returns steering and acceleration requests. Acceleration may be requested in $[-7,7]$ m/s², but the adapter clips it to the speed-dependent achievable range. [`dynamics.predict`](dynamics.py) includes that conversion for candidate-action evaluation. [INTERFACE.md](INTERFACE.md) defines the lifecycle, units, bounds, and scenario fields.

You may collect trajectories, improve the model, fit a value or policy approximation, or combine offline training with online improvement. You may also put a learning-based planner and your own tracker inside the same `Controller`; `act` must still return `{"steering", "acceleration"}`. Preserve the published evaluation interface. The base environment lacks optional Numba acceleration; measure runtime when evaluating many candidates. [Start here](START_HERE.md) shows a first predictor call, run-data check, and way to load a learned artifact.

```powershell
conda run --no-capture-output -n slalom2026 python .\run_local.py --mode full --controller .\my_controller.py --output .\runs\full
```

For reproducible comparisons, record the scenario, seed, mode, outcome, finish time, and controller version. `--config` selects a course JSON; the announced evaluation uses the shipped [`course.json`](course.json). For local spacing tests, copy it, set a new `scenario_id`, change `cone_spacing_m`, and place `finish_x_m` beyond the last cone. Keep the JSON with the result because `result.json` records its `scenario_id`, not every course parameter. Local experiments may instead specify `cone_centers_m` as eight `[x,y]` pairs in increasing X order (Y may vary); this overrides `cone_spacing_m`, and pass sides still alternate.
