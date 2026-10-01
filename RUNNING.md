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

The **planned REF position** at stage $k$ is $(X_k,Y_k)$, initially $(0,0)$, on a grid with $h_X=1$ m and $h_Y=0.5$ m. Every $\Delta=0.5$ s, choose integer moves $m_k\in\{1,2,3,4,5\}$ and $n_k\in\{-4,\ldots,4\}$. The position update is $X_{k+1}=X_k+m_k h_X$ and $Y_{k+1}=Y_k+n_k h_Y$.

The move implies virtual segment speed $V_k=\sqrt{(m_k h_X)^2+(n_k h_Y)^2}/\Delta$. Its global course angle is χ<sub>k</sub> = atan2(n<sub>k</sub> h<sub>Y</sub>, m<sub>k</sub> h<sub>X</sub>), with lateral displacement first and longitudinal displacement second.

These are not exact COM speed or body yaw. The basic geometric planning state is $(X_k,Y_k)$. If your DP enforces acceleration or turn limits, also carry the **previous segment's** $(V_{k-1},\chi_{k-1})$ as auxiliary memory, initialized to zero. The grid assumes each move is achieved exactly. PyChrono later tests the actual car and the supplied tracker.

The fixed grid and horizon come from [easy_spec.json](easy_spec.json); the finish position comes from [course.json](course.json). The failure cost $M$ is one possible DP design choice:

| Symbol | Value | Meaning |
|---|---:|---|
| $h_X$, $h_Y$ | 1 m, 0.5 m | Grid spacing |
| $\Delta$ | 0.5 s | Planning stage duration |
| $K_{\max}$ | 120 | Last allowed arrival stage |
| $X_F$ | 145 m | Finish line |
| $M$ | 1000 | Example failure cost; adjustable in your DP |

The table below gives **suggested starting bounds for your DP**, also listed in [easy_spec.json](easy_spec.json). You may tighten or relax them to explore faster plans. The plan validator does **not** enforce these seven numeric bounds; it enforces the action lattice, stage limit, gate sides, and first finish arrival. PyChrono still enforces its own fixed collision, road, speed, and heading rules. This simplified grid omits the vehicle footprint and tire and actuator dynamics, so a shorter DP plan may fail in PyChrono.

| Adjustable DP planning bound | Suggested starting value |
|---|---|
| Road strip | $\lvert Y_{k+1}\rvert\le3$ m |
| Segment speed | $V_k\le11.5$ m/s |
| Course angle | $\lvert\chi_k\rvert\le0.58$ rad |
| Acceleration | $(V_k-V_{k-1})/\Delta\le4$ m/s² |
| Deceleration | $(V_{k-1}-V_k)/\Delta\le6$ m/s² |
| Angle change | $\lvert\chi_k-\chi_{k-1}\rvert\le0.31$ rad |
| Virtual lateral acceleration | $V_k\lvert\chi_k-\chi_{k-1}\rvert/\Delta\le6$ m/s² |

**Every planned gate must be passed on its assigned side.** All basic-course cones have lateral position zero. At cone $j$'s X plane, linearly interpolate the planned segment to obtain $Y_{\mathrm{cross},j}$. The required signed offset is therefore $d_j=\sigma_jY_{\mathrm{cross},j}>0$, where $\sigma_j=+1$ at even-numbered cones (+Y pass) and $\sigma_j=-1$ at odd-numbered cones (−Y pass), starting with $j=0$. This gate-side rule is mandatory even if you relax all seven suggested planning bounds; a wrong-side plan is rejected before PyChrono runs. You may additionally require a chosen planning margin $d_j\ge d_{\mathrm{plan}}>0$. Neither $d_j>0$ nor a positive planning margin proves that the full vehicle will avoid the cone. For local courses with shifted cones, measure the crossing relative to each cone's lateral position as described in [INTERFACE.md](INTERFACE.md).

The first arrival at $X\ge X_F$ must occur by stage $K\le K_{\max}$. One time-minimizing DP uses stage cost $g(z_k,m_k,n_k)=1$ for each allowed move, including the finish move. Set $g(z_k,m_k,n_k)=M$ and terminate if a move violates the required gate side or your chosen DP bounds.

If you use speed and turn bounds, $z_k=(X_k,Y_k,V_{k-1},\chi_{k-1})$ includes the two constraint-memory values; otherwise $(X_k,Y_k)$ can suffice for the virtual gate problem. On first reaching $X_F$, terminate with cost **0**; if stage $K_{\max}$ ends before the finish, assign terminal cost **$M$**. A successful plan therefore costs its stage count $K$; failure costs at least $M$. All moves increase X, so backward DP is possible. Other course-based numerical methods are allowed if explained. The repository supplies the model, checker, and tracker, but no DP solver. The runner rejects a plan that violates a mandatory rule instead of simulating its penalty; $M$ is for your optimization formulation.

The effect of a chosen gate margin is visible in two backward-DP plans that used the table's suggested bounds as DP constraints. Both satisfy the virtual gate rule; their **signed planned offset $d_j$ equals the stated value at every gate**. The planned $Y_{\mathrm{cross},j}$ is $+d_j$ at even-numbered cones and $-d_j$ at odd-numbered cones.

| Chosen $d_j$ | Planned stages | PyChrono with the supplied tracker |
|---:|---:|---|
| 0.5 m | 31 (15.5 s virtual) | **Cone contact at 3.384 s, before gate 1** |
| 2.5 m | 48 (24.0 s virtual) | **Success:** 8 gates, finish 24.074 s; minimum scored footprint clearance 0.734 m |

The 0.5 m plan passed the *planning* gate-side test but its vehicle footprint hit a cone. The 2.5 m case succeeded in this run; a planning margin is a design choice, not a guarantee or an evaluation rule.

Save **all** planned 0.5 s moves, from the start through the first finish-line crossing, as one JSON object with exactly one `moves` list. Each entry is an integer pair `[m_k, n_k]`; the last pair must first reach or cross the finish. These complete, independent hand-designed examples show both outcomes without supplying a DP solver or the DP paths above:

| Complete plan file | Planned gate offset | PyChrono result |
|---|---|---|
| [Wide 73-stage plan](examples/easy_success_73.json) | 2.5 m at every gate | **Success:** 8 gates, finish 36.548 s; minimum footprint clearance 0.485 m |
| [Narrow 73-stage plan](examples/easy_failure_73.json) | 0.5 m at every gate | **Cone contact:** 5.341 s, before gate 1 |

The plans use the same longitudinal grid moves. Their differing lateral waypoints show why passing the virtual gate-side test does not establish physical clearance. They demonstrate the file interface; copying an example alone does not meet the assignment's course-based design requirement. Try either complete file directly:

```powershell
conda run --no-capture-output -n slalom2026 python .\run_local.py --mode easy --plan .\examples\easy_success_73.json --output .\runs\easy_success
conda run --no-capture-output -n slalom2026 python .\run_local.py --mode easy --plan .\examples\easy_failure_73.json --output .\runs\easy_failure
```

The second command intentionally returns a nonzero exit code after writing `result.json`. [easy_mode.py](easy_mode.py) checks the fixed lattice, horizon, planned gate sides, and first finish arrival; it converts accepted nodes to steering and pedals using the same tracker for everyone. The Full-mode acceleration request interface does not change Easy mode. Only PyChrono determines actual success, collision, and finish time.

### Full: vehicle controller

The supplied predictor uses state $s=(X_{\mathrm{REF}},Y_{\mathrm{REF}},\psi,v_x,v_y,r,\delta)$ and transition $\hat s_{k+1}=F_{\mathrm{red}}(s_k,u_k,u_{s,k-1}^{\mathrm{applied}};\Delta t=0.02\,\mathrm{s},\mu=0.9)$.

$v_x,v_y$ are body-frame COM velocities; $r$ is yaw rate; $\delta$ is measured mean front-wheel steering. The action $u_k=(u_{s,k}^{\rm req},a_{x,k}^{\rm req})$ requests steering and longitudinal acceleration. The runner limits the steering command sent to PyChrono to a change of **0.04 per 0.02 s**; the predictor includes that interface rule and a fitted steering lag, so prediction needs the previously applied steering. A fixed map converts acceleration requests to throttle or brake within speed-dependent limits. Use the model for model-based planning, policy improvement, or candidate-action prediction, then test the resulting controller in PyChrono. [MODEL.md](MODEL.md) explains the equations, coordinates, evidence, and validity range; [dynamics.py](dynamics.py) and [model_parameters.json](model_parameters.json) implement the fitted predictor.

To check the supplied model, [`validate_requests.py`](validate_requests.py) replays steering and acceleration requests recorded during Full-mode PyChrono runs through `dynamics.predict`. Each 2 s prediction starts from a measured PyChrono state; the table reports prediction error after those 2 s. This is a model check, not a controller function.

| PyChrono drive | Windows | Forward-speed RMSE | REF lateral-position RMSE |
|---|---:|---:|---:|
| Low-speed starter slalom | 58 | 0.3453 m/s | 0.0983 m |
| Straight drive to 10.74 m/s (calibration data) | 10 | 0.1184 m/s | 0.0777 m |

The straight-drive row checks longitudinal response at higher speed. Both rows use short prediction windows, so accuracy in a fast slalom or over the full course remains unverified. See the [comparison plot](validation/ax_request_comparison.png), [metrics](validation/ax_request_metrics.json), and [MODEL.md](MODEL.md) for the test conditions.

Copy [`controller.py`](controller.py) and implement `Controller.reset` and `Controller.act`. Every 0.02 s, `act` receives an observation and returns steering and acceleration requests. Acceleration may be requested in $[-7,7]$ m/s², but the adapter clips it to the speed-dependent achievable range. [`dynamics.predict`](dynamics.py) includes that conversion for candidate-action evaluation. [INTERFACE.md](INTERFACE.md) defines the lifecycle, units, bounds, and scenario fields.

You may collect trajectories, improve the model, fit a value or policy approximation, or combine offline training with online improvement. You may also put a learning-based planner and your own tracker inside the same `Controller`; `act` must still return `{"steering", "acceleration"}`. Preserve the published evaluation interface. The base environment lacks optional Numba acceleration; measure runtime when evaluating many candidates. [Start here](START_HERE.md) shows a first predictor call, run-data check, and way to load a learned artifact.

```powershell
conda run --no-capture-output -n slalom2026 python .\run_local.py --mode full --controller .\my_controller.py --output .\runs\full
```

For reproducible comparisons, record the scenario, seed, mode, outcome, finish time, and controller version. `--config` selects a course JSON; the announced evaluation uses the shipped [`course.json`](course.json). For local spacing tests, copy it, set a new `scenario_id`, change `cone_spacing_m`, and place `finish_x_m` beyond the last cone. Keep the JSON with the result because `result.json` records its `scenario_id`, not every course parameter. Local experiments may instead specify `cone_centers_m` as eight `[x,y]` pairs in increasing X order (Y may vary); this overrides `cone_spacing_m`, and pass sides still alternate.
