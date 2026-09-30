# Run and develop a controller

Complete [installation](INSTALL.md) first. Run all commands below from the repository root in a PowerShell or Miniforge Prompt. The local evaluator uses the same BMW E90/TMeasy PyChrono course and success rules in both modes; `--mode` changes only what you design and submit.

## Check the installation with the slow example

```powershell
conda run --no-capture-output -n slalom2026 python .\run_local.py --mode full --controller .\controller.py --output .\runs\starter
```

The supplied [`controller.py`](controller.py) follows an internally defined sinusoidal path with a **5 m/s** target speed and a simple proportional acceleration request. It is an interface and installation example, not a required reference path or a qualifying learning-based solution. In a headless PyChrono 10.0.0 check with this interface, it passed all eight gates and finished in **30.660 s**. Check your own `runs/starter/result.json` for `"status": "success"`, `gates_passed: 8`, and `finish_time_s`; small numerical differences are possible. Finish time is simulation time, not CPU time.

The runner is headless by default. To see the car and marked finish line, add `--visual` (a Windows graphical session is required). The viewer is for inspection; use headless runs for reported times. Results are written to `result.json` and `trajectory.npz` in the chosen output directory. `trajectory.npz` contains states and applied inputs sampled at control steps; see [interface details](INTERFACE.md).

## Choose a learning mode

| | Easy: fixed-time grid plan | Full: vehicle control |
|---|---|---|
| Your output | A sequence of integer grid moves in `easy_plan.json` | `Controller.reset` and `Controller.act` in `controller.py` |
| Planning/control interval | 0.5 s virtual planning stages; supplied tracker acts every 0.02 s | Your controller acts every 0.02 s |
| Supplied model | Exact virtual grid transitions and a fixed shared lower controller | Identified seven-state vehicle predictor and a shared acceleration-to-pedal adapter |
| Actual evaluation | Same PyChrono plant and course rules | Same PyChrono plant and course rules |

### Easy mode: a finite-horizon DP exercise

The planning state is $(i,j,m_{\mathrm{prev}},n_{\mathrm{prev}})$. Position $(X,Y)=(i h_X,j h_Y)$ is the **planned vehicle REF position**, with $h_X=1$ m and $h_Y=0.5$ m. The previous move is stored so acceleration and direction-change constraints can be checked. The initial state is $(0,0,0,0)$. Each 0.5 s stage selects integer $(m,n)$, where $m\in\{1,2,3,4,5\}$ and $n\in\{-4,\ldots,4\}$:

$$
i^+=i+m,\qquad j^+=j+n,\qquad
V=\frac{\sqrt{(m h_X)^2+(n h_Y)^2}}{0.5},\qquad
\chi=\operatorname{atan2}(n h_Y,m h_X).
$$

Here $V$ and $\chi$ are the virtual speed and global course angle of REF along one planned segment. They are **not** exactly the car's center-of-mass speed or body yaw. The model assumes the virtual grid move is achieved exactly; PyChrono then tests how well the shared lower controller can follow it. [`easy_spec.json`](easy_spec.json) contains every action and transition limit, including $|Y|\le3$ m, $V\le11.5$ m/s, $|\chi|\le0.58$ rad, acceleration $\le4$ m/s², deceleration $\le6$ m/s², angle change $\le0.31$ rad, and virtual lateral acceleration $V|\Delta\chi|/0.5\le6$ m/s². At each cone's X plane the planned line segment must cross on the designated side. The virtual plan does not represent the vehicle's full footprint; choose your own safety margin and verify clearance in PyChrono.

The first planned arrival at $X\ge145$ m must occur by stage $K\le120$. Give **every feasible move** stage cost 1, including the move that reaches the finish. The successful terminal state adds no cost; an invalid move or an unfinished plan at the horizon incurs a penalty $M=1000$. Thus a successful plan costs exactly its **arrival stage $K$**, while a failed plan costs at least $M$. Because all valid moves increase X, the finite grid permits backward DP. A different numerical method is also acceptable; your report should explain your own method. The repository supplies the model, checker, and lower controller, but no DP solver or optimal move sequence.

For the side-only virtual gate rule, backward DP found a **31-stage** grid path (15.5 s planned). Its planned lateral separation at each cone is only 0.5 m, so the grid result alone does **not** establish a safe PyChrono run. A separate, deliberately conservative plan using a 2.5 m planning margin took **48 stages** (24.0 s planned); the shared tracker completed the PyChrono course in **24.074 s** and passed all eight gates. Its saved trajectory remains clear under the published COM-centered scoring footprint. The margin is a planning choice in this example, **not an evaluation gate rule**. No claim of real-vehicle optimality follows from either grid result. Tracking error and omitted tire/actuator state mean a grid-feasible plan can still fail in PyChrono.

Write a JSON file with your own full move list:

```json
{"moves": [[1, 0], [2, 0]]}
```

The two entries above are feasible initial moves and illustrate the JSON format, but they do not reach the finish. The runner therefore rejects this incomplete file; replace it with your own complete plan before running. Each entry must contain two integers. The last entry must be the **first** move that reaches or crosses the finish. Run your complete plan with:

```powershell
conda run --no-capture-output -n slalom2026 python .\run_local.py --mode easy --plan .\easy_plan.json --output .\runs\easy
```

The validator checks the lattice, transition limits, and planned gate crossings. [`easy_mode.py`](easy_mode.py) then turns the plan into steering and pedal commands using the same lower controller for everyone. The full-mode acceleration request interface does not change this fixed easy-mode tracker. Actual success, collision, and finish time come only from PyChrono.

### Full mode: design the vehicle controller

The supplied predictor has state

$$
s=(X_{\mathrm{REF}},Y_{\mathrm{REF}},\psi,v_x,v_y,r,\delta),
\qquad \dot s=f_{\mathrm{red}}(s,u;\mu=0.9),
$$

where $v_x,v_y$ are body-frame COM velocities, $r$ is yaw rate, and $\delta$ is measured mean front-wheel steering angle. The full-mode action $u=(u_s^{\rm req},a_x^{\rm req})$ contains a steering request and a desired longitudinal acceleration in m/s². The supplied open-loop adapter converts acceleration requests to throttle or brake; its achievable acceleration depends on speed. [`MODEL.md`](MODEL.md) explains the tire forces, actuator approximation, steering dynamics, REF/COM relationship, and validity range. [`dynamics.py`](dynamics.py) and [`model_parameters.json`](model_parameters.json) are the numerical implementation. The supplied coefficients were fitted at friction 0.9 and are **not** an exact PyChrono model.

The comparison [plot](validation/model_comparison.png) and [`metrics.json`](validation/metrics.json) use fixed-input predictions restarted from measured states at the beginning of each window. For a held-out mixed maneuver, the 2 s endpoint RMSE was **0.0832 m/s** in forward speed and **0.0382 m** in REF lateral position. For a baseline course maneuver included in fitting, the corresponding errors were **0.2401 m/s** and **0.0560 m**. A separate held-out aggressive slalom maneuver reached **0.4439 m/s** and **0.2111 m**, respectively; [MODEL.md](MODEL.md) explains this summary and its limits. These short-window errors do not establish accurate full-course open-loop prediction or reliability at the handling limit. Run `validate_model.py` to reproduce the public-trace plot and metrics.

Implement your controller in a copy of [`controller.py`](controller.py). It receives the current observation and returns steering and acceleration requests every 0.02 s. The acceleration request is allowed in $[-7,7]$ m/s², but the adapter limits the effective value to what the identified actuator can produce at the current speed. Use [`dynamics.predict`](dynamics.py) to include this same conversion in model-based DP, PI, or rollout. The exact lifecycle, units, bounds, and scenario fields are in [INTERFACE.md](INTERFACE.md). You may collect your own trajectories using the local runner, improve or replace the supplied model, fit a value or policy approximation, or combine offline training with online improvement. Preserve the published action and evaluation interface in your submitted code.

You may organize a Full-mode solution hierarchically. For example, a learning-based upper-level planner can choose a path or speed target while your own lower-level tracking controller computes the required steering and acceleration. Put both parts inside your submitted `Controller`; `act` must still return the published `{"steering", "acceleration"}` action. The supplied open-loop adapter then converts the acceleration request to throttle or brake. This choice does not change the Easy-mode tracker or its grid-planning task.

```powershell
conda run --no-capture-output -n slalom2026 python .\run_local.py --mode full --controller .\my_controller.py --output .\runs\full
```

For reproducible comparisons, record the scenario, seed, mode, outcome, finish time, and controller version. `--config` selects a local course JSON file; the basic scored course is [`course.json`](course.json). For local spacing experiments, copy it, change `cone_spacing_m`, and move `finish_x_m` beyond the last cone. If you explicitly provide `cone_centers_m`, use eight ordered `[x,y]` pairs with the same Y coordinate; the official bonus varies cone spacing only. The same alternating pass directions apply. The bonus spacing range and scoring will be announced separately. The basic and bonus runs must use the same controller and learned object; the spacing value may be read by that unchanged controller or learned object.
