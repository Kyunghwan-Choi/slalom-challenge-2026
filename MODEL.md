# Supplied control-oriented vehicle model

The evaluator uses PyChrono 10.0.0's BMW E90 with TMeasy tires on level rigid terrain. PyChrono drives the car through steering, throttle, and brake inputs. Full mode asks you for **steering and longitudinal-acceleration requests** instead; a supplied fixed conversion sends throttle or brake to PyChrono. This simplifies controller design. [`dynamics.py`](dynamics.py) predicts the response with a reduced planar bicycle model fitted at road friction $\mu=0.9$. It does not reproduce every PyChrono state or guarantee behavior at untested limits. See [Project Chrono's vehicle manual](https://api.projectchrono.org/manual_vehicle.html) for the underlying framework.

![REF, COM, vehicle heading, and the scored car footprint](assets/ref_com.svg)

Global axes are capital $X,Y$; rotating body axes are lowercase $x,y$.

## Coordinates and state

The seven-state vector is

$$
s=(X_{\rm REF},Y_{\rm REF},\psi,v_x,v_y,r,\delta).
$$

![Physical meaning of the seven model states: REF position, heading, body velocities, yaw rate, and front-wheel steering](assets/state_variables.svg)

Global +X follows the course and +Y points left. Body $+x$ points forward and $+y$ left. $p_{\rm REF}=(X_{\rm REF},Y_{\rm REF})$ and $p_{\rm COM}=(X_{\rm COM},Y_{\rm COM})$ are **global** positions. Heading $\psi$ is counterclockwise from global +X to body $+x$; $v_x,v_y$ are body-frame COM velocities, $r$ is yaw rate, and $\delta$ is measured mean front-wheel steering. REF lies $a_{\rm REF}=1.371$ m ahead of COM:

$$
p_{\rm REF}=p_{\rm COM}+a_{\rm REF}(\cos\psi,\sin\psi),
$$
$$
\dot X_{\rm REF}=v_x\cos\psi-(v_y+a_{\rm REF}r)\sin\psi,\qquad
\dot Y_{\rm REF}=v_x\sin\psi+(v_y+a_{\rm REF}r)\cos\psi,\qquad
\dot\psi=r.
$$

The $a_{\rm REF}r$ term matters in a slalom. Gates and finish use REF; collision scoring uses a **4.7 m × 1.9 m** body-aligned rectangle centered at COM. No third vehicle point is used. A COM planner must transform predicted poses to REF for gate/finish checks and test the oriented footprint against cones and road edges. One fixed COM clearance margin cannot exactly replace those pose-dependent checks.

Mass is $m=1909.883$ kg and wheelbase is $L=2.776$ m. [`model_parameters.json`](model_parameters.json) holds the wheelbase split, steering map, tire stiffness, yaw inertia, and other **fitted effective coefficients**, not independently measured vehicle specifications.

## Dynamics and actuator

The Full-mode action is $u=(u_s^{\rm req},a_x^{\rm req})$: dimensionless steering $|u_s^{\rm req}|\le0.8$ and requested longitudinal acceleration $|a_x^{\rm req}|\le7$ m/s². The runner limits changes in the steering input sent to PyChrono to **0.04 per 0.02 s**. This is an input rule that prevents abrupt commands, **not** a physical lag identified from PyChrono. The model also includes the measured wheel-angle lag below; its one-step prediction needs the previously applied steering input. The same steering input rule applies in Easy mode.

At the current $v_x$, a fixed **open-loop conversion** chooses throttle $u_t$ or brake $u_b$, each in $[0,0.6]$ and never both nonzero. For model-based control, distinguish the requested $a_x^{\rm req}$ from the reduced model's predicted longitudinal acceleration term $\hat a_x(v_x,u_t,u_b;\mu)$. For forward motion ($v_x\ge0$), at the **start** of step $k$ the conversion chooses pedals so that the fitted static map satisfies

$$
\hat a_x(v_{x,k},u_{t,k},u_{b,k};\mu)=
\operatorname{clip}\!\left(a_{x,k}^{\rm req},a_{\min}(v_{x,k}),a_{\max}(v_{x,k})\right).
$$

The bounds are speed dependent and reported by [`contract.acceleration_bounds`](contract.py). The fitted inverse is algebraically one-to-one at fixed forward speed within those bounds, but **PyChrono may accelerate differently**. If the car rolls backward, the adapter instead commands braking to recover. Pedals stay fixed for 0.02 s, so even the model's $\hat a_x$ may change slightly during a step as $v_x$ changes. There is no feedback correction of acceleration in this conversion. [`dynamics.predict`](dynamics.py) includes it: model-based DP, PI, or rollout can compare requested actions without implementing a pedal map.

The main lateral and yaw equations are

$$
\dot v_x=\hat a_x+r v_y-\frac{F_{yf}\sin\delta}{m},\qquad
\dot v_y=\frac{F_{yf}\cos\delta+F_{yr}}{m}-r v_x,
$$
$$
\dot r=\frac{l_fF_{yf}\cos\delta-l_rF_{yr}}{I_z},\qquad l_f+l_r=L.
$$

Front/rear slip-angle approximations are

$$
\alpha_f=\delta-\operatorname{atan2}(v_y+l_fr,\max(v_x,1)),\qquad
\alpha_r=-\operatorname{atan2}(v_y-l_rr,\max(v_x,1)).
$$

Lateral force is $F_{yi}=F_{i,\mathrm{cap}}\tanh(C_i\alpha_i/F_{i,\mathrm{cap}})$; capacity depends on friction, static axle load, and simplified sharing with longitudinal force. This smooth fit is not an exact tire friction ellipse. The first equation shows why $\hat a_x$ is **not** $\dot v_x$ during a turn. The fitted longitudinal term is

$$
\hat a_x=a_D(v_x,u_t;\mu)-a_B(v_x,u_b;\mu)
-d_0\tanh(v_x/0.2)-d_2v_x|v_x|+0.35R(v_x),
$$

where $a_D$ and $a_B$ are the fitted propulsion and braking acceleration terms. Their smooth saturation and coefficients are in [`contract.py`](contract.py), [`dynamics.py`](dynamics.py), and [`model_parameters.json`](model_parameters.json). $R$ is the high-speed correction defined below.

Measured steering follows a fitted speed-dependent *linear* gain and first-order lag:

$$
\dot\delta=\frac{(s_g+s_vv_x^2)u_s-\delta}{\tau_s},
\qquad(s_g,s_v,\tau_s)=(0.551426,-0.00042031,0.007088\ {\rm s}).
$$

Below **2 m/s**, the implementation blends the lateral/yaw equations with a kinematic bicycle response. [`dynamics.py`](dynamics.py) defines capacity floors, smoothing, and midpoint integration. `dynamics.predict(state, command, previous_steering, scenario)` applies the adapter and predicts one **0.02 s** step from a `{"steering", "acceleration"}` request. Lower-level `dynamics.step` and `dynamics.replay` instead take applied `[steering, throttle, brake]` inputs for reproducible validation traces. `dynamics.parameters()` rejects friction other than **0.9** because other surfaces were not validated. Actual PyChrono acceleration may differ, especially outside measured driving conditions.

### High-speed correction

The longitudinal term $\hat a_x$ above has two corrections. The propulsion term $a_D$ has the form $D\tanh(g(v)u_t^{q_t}/D)$, with drive gain

$$
v=\max(v_x,0),\qquad
g(v)=\max\{0.1,\ a_0+a_1v+a_2v^2,\ 5\mathbf{1}_{\{v\ge9.2\}}\}.
$$

The added $0.35R(v_x)$ in $\hat a_x$ changes modeled coast/drag. Here $R=0$ below **9.2 m/s**, $R=1$ above **9.5 m/s**, and $R=3z^2-2z^3$ between them with $z=(v_x-9.2)/0.3$. These terms are **longitudinal**; the separate $s_vv_x^2$ term in the steering equation changes steering gain. Straight drive and coast traces calibrated the longitudinal corrections after the earlier fit underestimated propulsion near the **12 m/s** course limit. The same corrected map is used for pedal conversion and prediction.

The correction does not guarantee requested acceleration. In a PyChrono straight run near **10.5 m/s**, $a_x^{\rm req}=0$ still increased speed by **0.207 m/s over 2.4 s**. Leave speed margin and test your controller in PyChrono near the 12 m/s failure limit.

## Comparison with PyChrono

![Forward-speed response to logged Full-mode steering and acceleration requests](validation/ax_request_comparison.png)

The main check replays **logged Full-mode steering and $a_x^{\rm req}$ commands** through [`dynamics.predict`](dynamics.py), including its steering limit and pedal conversion. Every prediction window starts from a measured PyChrono state and runs open loop for 0.5, 1, or 2 s. Windows start 0.5 s apart and can overlap. The plot shows the 2 s forward-speed endpoints.

| PyChrono drive | Speed range | Horizon | Forward-speed endpoint RMSE | REF lateral endpoint RMSE |
|---|---:|---:|---:|---:|
| Supplied 5 m/s slalom | 0.17–5.14 m/s | 0.5 s | 0.0873 m/s | 0.0056 m |
| Supplied 5 m/s slalom | 0.17–5.14 m/s | 1.0 s | 0.1732 m/s | 0.0210 m |
| Supplied 5 m/s slalom | 0.17–5.14 m/s | 2.0 s | **0.3453 m/s** | **0.0983 m** |
| High-speed straight probe | 0.17–10.74 m/s | 0.5 s | 0.0947 m/s | 0.0053 m |
| High-speed straight probe | 0.17–10.74 m/s | 1.0 s | 0.1045 m/s | 0.0198 m |
| High-speed straight probe | 0.17–10.74 m/s | 2.0 s | **0.1184 m/s** | **0.0777 m** |

The slalom is the supplied example controller's run; the straight probe is related to longitudinal calibration and is **not an independent fast-slalom test**. These figures compare the **response to requested actions** rather than assuming $a_x^{\rm req}=\dot v_x$; in a turn, lateral forces also affect $\dot v_x$. The 2 s starter-speed bias is material for tight cone clearance. Reproduce the table and plot using [`validate_requests.py`](validate_requests.py), the [slalom trace](validation/request_starter.npz), and the [straight trace](validation/request_straight.npz); the numerical summary is in [`ax_request_metrics.json`](validation/ax_request_metrics.json). Matplotlib is needed only to regenerate the plot.

For a separate check of the vehicle equations alone, the repository also includes [fixed-input validation metrics](validation/metrics.json), [plots](validation/model_comparison.png), and [`validate_model.py`](validate_model.py). Those tests give the model exactly the same applied steering and pedals as PyChrono, so their errors do not measure the complete Full-mode command response.

These are **short-window** tests. Full-course open-loop error accumulates; accuracy for fast slaloms, large sideslip, wheel spin or lock, other friction, and extreme student policies is unverified. The model omits PyChrono's full roll/pitch, gear, engine, wheel, and tire states. Validate the final controller in PyChrono.
