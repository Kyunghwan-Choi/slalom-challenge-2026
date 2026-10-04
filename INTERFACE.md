# Input and output interface

Both modes use the same PyChrono course. Choose `--mode easy|full`: Easy reads `--plan`; Full loads `--controller`. See [RUNNING.md](RUNNING.md) for the learning tasks. Physical quantities use SI units; steering requests and native PyChrono driver inputs are dimensionless.

The **runner**, [run_local.py](run_local.py), executes each episode, sends observations to the controller, advances the vehicle, checks driving rules, and saves results. Its **input adapter**, [contract.py](contract.py), checks commands and converts them to native vehicle inputs. The [system diagrams](RUNNING.md#how-the-system-works) show this feedback loop and the separate Easy plan-validation step.

## Easy plan

Submit a UTF-8 JSON object with exactly one key, `moves`, containing a nonempty list of integer `[m,n]` pairs. The **last move must be the first** to reach `finish_x_m`. The [complete successful plan](examples/easy_success_73.json) and [complete physical-failure plan](examples/easy_failure_73.json) show the exact format; both pass the virtual plan checks. [`easy_spec.json`](easy_spec.json) defines the fixed grid and suggested DP bounds. [`easy_mode.py`](easy_mode.py) checks the lattice, horizon, gate sides, and finish, then converts accepted nodes to steering and pedal commands. The suggested speed, turn, and road-strip bounds may be adjusted in a student's DP and are not plan-validation rules. PyChrono applies the fixed physical safety rules. Easy submissions do not implement `Controller.act` or use Full mode's acceleration request.

## Full controller

Create a `Controller` class in the file passed to `--controller`:

```python
class Controller:
    def reset(self, scenario: dict, seed: int) -> None:
        self.scenario = scenario

    def act(self, observation: dict) -> dict:
        return {"steering": 0.0, "acceleration": 1.0}
```

The runner constructs a new controller and calls `reset` once per episode, after **0.8 s** of vehicle settling. The first `act` call has `time=0`; calls then occur every **0.02 s** until success or failure. Resolve data and weights relative to `Path(__file__).resolve().parent`, not the terminal directory. `seed` supports reproducible stochastic calculations; the basic local course and initial procedure are deterministic.

`scenario` includes [`course.json`](course.json) entries, `mode="full"`, and `cones`. Each cone has metre-valued `x`, `y`, and `pass_sign`: `+1` requires a +Y pass, `-1` a −Y pass. The dictionary also provides finish, road and gate geometry, limits, and friction. No reference trajectory is supplied.

For local variations, copy `course.json`, assign a new `scenario_id`, change `cone_spacing_m`, and move `finish_x_m` beyond the last cone. Alternatively, add `"cone_centers_m": [[x_0,y_0], ..., [x_7,y_7]]`: eight finite pairs whose X values strictly increase and precede the finish; Y may vary. The list overrides `cone_spacing_m` for cone placement; pass sides still alternate. Keep the copied JSON with results. Only the shipped `course.json` is used for the announced evaluation; these options support local experiments.

### Observation

| Key | Value |
|---|---|
| `schema_version` | `"2.0"` |
| `time`, `dt` | Seconds since settling; control interval |
| `state` | Seven elements in the order below |
| `next_gate` | Zero-based index of first unpassed cone; 8 after all gates |
| `previous_applied` | Last dimensionless `steering`, `throttle`, `brake` inputs after the input adapter |
| `remaining_time` | Seconds until the 60 s simulation limit |

| Index | `state` variable | Meaning |
|---:|---|---|
| 0 | `X_ref` | Global REF X, m |
| 1 | `Y_ref` | Global REF Y, m |
| 2 | `psi` | Counterclockwise body heading, rad |
| 3 | `vx_COM` | Body-frame forward COM speed, m/s |
| 4 | `vy_COM` | Body-frame leftward COM speed, m/s |
| 5 | `yaw_rate` | Body yaw rate, rad/s |
| 6 | `mean_front_steer` | Measured mean front-wheel steering angle, rad |

Global +X follows the course and +Y points initially left. `vx_COM` is not global X progress speed. REF lies **1.371 m** ahead of COM along the body axis; the scored rectangular footprint is centered at COM. [MODEL.md](MODEL.md) gives the coordinate relation. PyChrono engine, gear, wheel, and tire internal states are absent; store any needed history in the controller object.

### Action and input adapter

`act` must return **exactly** `{"steering": number, "acceleration": number}`. Both must be finite Python `int` or `float` values within range. Cast NumPy values with `float(...)`. Booleans, missing or extra keys, NaN, infinity, and out-of-range requests cause `controller_error`.

| Key | Range | Meaning |
|---|---:|---|
| `steering` | `[-0.8, 0.8]` | Dimensionless request; positive turns left |
| `acceleration` | `[-7, 7]` m/s² | Desired longitudinal acceleration term $a_x^{\rm req}$; positive propels, negative brakes |

The input adapter limits applied steering changes to **0.04 per 0.02 s** from the previous applied request. It uses observed $v_x$ and a fitted, speed-dependent **open-loop inverse map** to select throttle or brake in $[0,0.6]$; it never commands both. Requests beyond pedal-achievable acceleration are clipped to the nearest feasible value. `acceleration=0` requests modeled zero longitudinal acceleration after resistance, not coasting. There is no acceleration feedback loop. Actual front-wheel angle (`state[6]`) lags the request.

[`contract.apply_action`](contract.py) implements the input adapter; `acceleration_bounds(vx, scenario)` returns the model's feasible interval. Its inverse is exact for the fitted static pedal map after clipping, but measured PyChrono acceleration may differ because the fit omits vehicle state and dynamics. The desired $a_x$ is not exactly $\dot v_x$ in a turn; see [MODEL.md](MODEL.md).

[`dynamics.predict(state, command, previous_steering, scenario)`](dynamics.py) applies the same input adapter and returns `(predicted_next_state, applied_inputs)`, where `applied_inputs` is `[steering, throttle, brake]`. Use it for candidate high-level actions. Lower-level `dynamics.step` and `dynamics.replay` take **applied native inputs** directly, as do the validation traces. In `act`, use the observation's steering memory:

```python
from dynamics import predict
next_state, applied_inputs = predict(
    observation["state"], candidate_action,
    observation["previous_applied"]["steering"], self.scenario
)
```

## Outputs and timing

A completed runner call writes `result.json` and `trajectory.npz`. Plan-validation, controller import/reset, or simulator errors can stop before these files are written; check the terminal error.

| File | Contents |
|---|---|
| `result.json` | `scenario_id`, `mode`, `status`, `finish_time_s` (`null` on failure), `gates_passed`, `gate_crossings`, `last_cone_time_s`, `min_clearance_m`, `initial_state`, `final_state`, action-call latency summaries; Easy also records `planned_stages` |
| `trajectory.npz` | `rows`: each `[state (7), applied steering, applied throttle, applied brake, elapsed simulation time]`; Full also has `requested_actions`, one `[steering_request, acceleration_request]` per row |

For Full row `k`, `rows[k]` stores the **state after** `requested_actions[k]`. Its pre-action state is `result.json`'s `initial_state` for `k=0`, otherwise the preceding row's state. The requested actions and applied inputs are both available for offline training. An event may fall between 0.02 s samples; use `result.json` for local event measurements, not the last trajectory row.

Finish time is interpolated at the **first forward REF crossing** of `finish_x_m` after all gates. Gate crossings are interpolated at cone X planes. Cone, road, and vehicle limits are checked every **0.001 s** physics step. `last_cone_time_s` is the eighth gate crossing, not finish time. At cone $j$, the crossing must satisfy $\sigma_j(Y_{\rm REF}(t_j)-Y_j)>0$; $\sigma_j$ is its required pass-side sign and $Y_j$ is its center's lateral position. Equality or the wrong side fails. There is no fixed lateral gate distance, and cone contact and footprint checks are separate.

The local runner records wall-clock action latency but imposes no computation deadline (`timing_enforced: false`); the evaluation budget will be announced separately. Evaluation imports must work in the supplied Conda environment; see [submission requirements](SUBMISSION_AND_AI.md).
