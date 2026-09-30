# Input and output interface

The local runner offers `easy` and `full` modes on the same PyChrono course. Choose the mode with `--mode easy|full`. Easy mode reads a complete grid plan from `--plan`; full mode loads a Python controller from `--controller`. [RUNNING.md](RUNNING.md) explains both learning problems. Physical quantities use SI units; the steering request and internal PyChrono driver inputs are dimensionless.

## Easy-mode plan file

Submit a UTF-8 JSON object with exactly one key, `moves`. Its value is a nonempty list of integer pairs `[m,n]`:

```json
{"moves": [[1, 0], [2, 0]]}
```

The snippet shows two feasible initial moves, not a full-course solution; the runner rejects it because it does not reach the finish. [`easy_spec.json`](easy_spec.json) and [`easy_mode.py`](easy_mode.py) define the fixed grid, action and transition limits, and plan validator. The plan must first reach `finish_x_m` on its last move. The runner's shared lower controller turns accepted plan nodes directly into steering and pedal commands; easy-mode submissions do not implement `Controller.act` or use the full-mode acceleration request interface.

## Full-mode controller lifecycle

Create a `Controller` class in the file supplied to `--controller`:

```python
class Controller:
    def reset(self, scenario: dict, seed: int) -> None:
        self.scenario = scenario

    def act(self, observation: dict) -> dict:
        return {"steering": 0.0, "acceleration": 1.0}
```

A new controller is constructed and `reset` is called once per episode, after the common 0.8 s vehicle settling period. The first `act` call has `time=0`; then it is called at 0.02 s intervals until success or failure. Read data or weights relative to `Path(__file__).resolve().parent`, not the current terminal directory. The `seed` supports your reproducible stochastic calculations; the basic local course and initial procedure are deterministic.

The `scenario` dictionary contains the entries of [`course.json`](course.json), `mode="full"`, and a `cones` list. Each cone has `x` and `y` coordinates in metres and `pass_sign`, which is `+1` for a required +Y pass or `-1` for a required −Y pass. `finish_x_m`, road and gate geometry, limits, and friction are also present. A single unchanged controller may read the spacing parameter for bonus scenarios. A reference trajectory is **not** supplied.

For local spacing experiments, copy the course file, assign a new `scenario_id`, and change `cone_spacing_m` and `finish_x_m`. You may also specify `"cone_centers_m": [[x_0,y_0], ..., [x_7,y_7]]` for nonuniform local experiments; X coordinates must strictly increase and precede the finish, while Y coordinates may vary. When present, this list overrides `cone_spacing_m` for cone placement. The required pass side still alternates with cone index. The basic scored layout remains the shipped [`course.json`](course.json); the official **Full-mode-only** bonus changes cone spacing only, and its distribution will be announced separately. Keep the copied course JSON with your run results. Keep the Full-mode controller code and learned object or weights identical across basic and bonus runs; the changed spacing may be passed to them as an input.

### Observation

| Key | Meaning |
|---|---|
| `schema_version` | `"2.0"` |
| `time`, `dt` | Elapsed simulation time after settling and control interval, in seconds |
| `state` | Seven-element list specified below |
| `next_gate` | Zero-based index of the first unpassed cone; 8 after all eight gates |
| `previous_applied` | Last applied dimensionless `steering`, `throttle`, and `brake` driver inputs, after the adapter |
| `remaining_time` | Time to the 60 s simulation-time limit, in seconds |

`state` has this fixed order:

| Index | Variable | Meaning and unit |
|---:|---|---|
| 0 | `X_ref` | Global X position of the vehicle reference point REF, m |
| 1 | `Y_ref` | Global Y position of REF, m |
| 2 | `psi` | Body heading, counterclockwise positive, rad |
| 3 | `vx_COM` | Forward velocity at COM, expressed in the body frame, m/s |
| 4 | `vy_COM` | Leftward velocity at COM, expressed in the body frame, m/s |
| 5 | `yaw_rate` | Body yaw rate, rad/s |
| 6 | `mean_front_steer` | Measured mean front-wheel steering angle, rad |

Global +X is the course direction, and +Y is initially left of the car. `vx_COM` is **not** global X progress speed. REF is ahead of COM by 1.371 m along the body axis; the scored rectangular footprint is centered at COM. [MODEL.md](MODEL.md) gives the planar coordinate relation. The observation does not include the complete PyChrono engine, gear, wheel, or tire internal state. If your algorithm needs a history, store it in the controller object between calls.

### Action and actuator adapter

`act` returns **exactly** `{"steering": number, "acceleration": number}`. Both entries must pass the finite Python `int`/`float` check; cast NumPy results with `float(...)` to avoid type-dependent behavior. Booleans, missing or extra keys, NaN, infinity, and requests outside the command ranges cause `controller_error`. This validation is separate from physical actuator saturation below.

| Key | Range | Interpretation |
|---|---:|---|
| `steering` | `[-0.8, 0.8]` | Dimensionless steering request; positive turns left |
| `acceleration` | `[-7, 7]` m/s² | Desired longitudinal acceleration term $a_x^{\rm req}$; positive requests propulsion, negative requests braking |

The adapter changes applied steering by at most **0.04 per 0.02 s** relative to the previous applied request. It uses the observed $v_x$ and an identified, speed-dependent **open-loop inverse map** to turn `acceleration` into throttle or brake, each in $[0,0.6]$; they are never commanded together. There is no acceleration feedback loop in this adapter. At each speed, a requested acceleration outside the pedal-achievable interval is saturated to the closest achievable value. In particular, `acceleration=0` requests zero longitudinal acceleration after modeled resistance, rather than coast. The actual wheel angle is observed in `state[6]` and lags the steering request.

[`contract.apply_action`](contract.py) is the runner's full-mode conversion; its `acceleration_bounds(vx, scenario)` helper gives the model's speed-dependent feasible interval. The inverse is exact for its fitted static pedal map after saturation, but measured PyChrono acceleration can differ because that map is only an approximation and the vehicle has unmodeled states. The supplied [`dynamics.predict(state, command, previous_steering, scenario)`](dynamics.py) applies the same adapter and returns `(predicted_next_state, applied_inputs)`, where `applied_inputs` is `[steering, throttle, brake]`. Use this high-level prediction interface when comparing candidate actions in model-based control. The lower-level `dynamics.step` and `dynamics.replay` continue to take those **applied native inputs** directly, as do the provided validation traces. The desired $a_x$ is a model term, not exactly $\dot v_x$ during a turn; see [MODEL.md](MODEL.md).

For a one-step candidate in `act`, pass the applied steering memory from the observation:

```python
from dynamics import predict
next_state, applied_inputs = predict(
    observation["state"], candidate_action,
    observation["previous_applied"]["steering"], self.scenario
)
```

## Outputs and evaluation timing

The chosen output folder contains `result.json` and `trajectory.npz`. `result.json` records `status`, `finish_time_s` (`null` on failure), `gates_passed`, `gate_crossings`, `last_cone_time_s`, `min_clearance_m`, initial and final state, mode, and action-call latency summaries. Easy runs additionally record `planned_stages`. `trajectory.npz` contains `rows`; each sampled row is `[state (7), applied steering, applied throttle, applied brake, elapsed simulation time]`. In full mode it also contains `requested_actions` with one `[steering_request, acceleration_request]` pair per row, so you can use the requested physical actions in offline training while retaining the actual applied driver inputs. Row `k` stores the **state after** applying `requested_actions[k]`; its pre-action state is `result.json`'s `initial_state` for `k=0`, or the preceding row's state otherwise. An event can terminate between 0.02 s samples, so the last row need not be the exact finish state; use `result.json` for official local event measurements.

Finish time is interpolated from the **first forward REF crossing** of `finish_x_m` after all gates. Gate crossing is likewise interpolated at each cone's X plane. Cone/road/vehicle constraints are checked during 0.001 s physics substeps. `last_cone_time_s` is the eighth gate crossing, **not** finish time. The local runner records wall-clock action latency but does not enforce a computation deadline (`timing_enforced: false`); the evaluation computation budget will be announced separately.
At a gate, `pass_sign * (Y_ref_cross - Y_cone) > 0` is required. Equality or crossing on the wrong side fails; no additional fixed lateral distance is imposed. The physical cone-contact and scored-footprint tests remain separate.
