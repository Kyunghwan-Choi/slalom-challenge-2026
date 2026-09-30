# Supplied control-oriented vehicle model

The evaluation plant is PyChrono 10.0.0's BMW E90 with TMeasy tires on level rigid terrain. The supplied [`dynamics.py`](dynamics.py) is a **reduced predictor** identified from that plant at road friction coefficient $\mu=0.9$. It is useful for model-based design and comparison, but it is not the PyChrono evaluator or a guarantee of vehicle behavior at untested limits. The model structure combines planar nonlinear bicycle dynamics and a steering actuator. A supplied open-loop lower controller translates desired longitudinal acceleration into the vehicle's native pedals. [Project Chrono's vehicle manual](https://api.projectchrono.org/manual_vehicle.html) describes the underlying vehicle and tire framework.

![REF, COM, vehicle heading, and the scored car footprint](assets/ref_com.svg)

The capital $X,Y$ axes are global; the lowercase $x,y$ axes rotate with the vehicle.

## Coordinates and state

The seven-state vector is

$$
s=(X_{\rm REF},Y_{\rm REF},\psi,v_x,v_y,r,\delta).
$$

The following sketch locates each state component on the vehicle and course.

![Physical meaning of the seven model states: REF position, heading, body velocities, yaw rate, and front-wheel steering](assets/state_variables.svg)

Global +X points along the course and global +Y points left. Body $+x$ points forward and $+y$ points left; these vehicle-fixed axes rotate with the car. $p_{\rm REF}=(X_{\rm REF},Y_{\rm REF})$ and $p_{\rm COM}=(X_{\rm COM},Y_{\rm COM})$ are both expressed in the **global** frame. The heading $\psi$ is measured counterclockwise from global +X to body $+x$; $r$ is yaw rate. $v_x,v_y$ are COM velocities expressed in the body frame, and $\delta$ is the measured mean front-wheel steering angle. The REF point is $a_{\rm REF}=1.371$ m ahead of COM along body $+x$. Thus

$$
p_{\rm REF}=p_{\rm COM}+a_{\rm REF}(\cos\psi,\sin\psi),
$$
$$
\dot X_{\rm REF}=v_x\cos\psi-(v_y+a_{\rm REF}r)\sin\psi,\qquad
\dot Y_{\rm REF}=v_x\sin\psi+(v_y+a_{\rm REF}r)\cos\psi,\qquad
\dot\psi=r.
$$

The $a_{\rm REF}r$ term matters in a slalom. REF does **not** follow COM kinematics without correction. The 4.7 m × 1.9 m scored collision rectangle is centered at COM and rotates with $\psi$; no third vehicle point is used. Gate and finish crossings are evaluated at REF. A COM-based planner is possible if it transforms the predicted pose to REF for gate/finish checks and tests the oriented car footprint against cones and road edges. A single fixed COM clearance margin cannot reproduce those pose-dependent checks exactly.

The car mass is $m=1909.883$ kg and wheelbase $L=2.776$ m. The numerical wheelbase split, steering map, tire stiffness, yaw inertia, and other parameters are in [`model_parameters.json`](model_parameters.json). Parameter values are fitted effective coefficients, not independently measured vehicle specifications.

## Dynamics and actuator inputs

The full-mode action is $u=(u_s^{\rm req},a_x^{\rm req})$: a dimensionless steering request and a **desired longitudinal acceleration** in m/s². Requests satisfy $|u_s^{\rm req}|\le0.8$ and $|a_x^{\rm req}|\le7$ m/s². The public [action adapter](INTERFACE.md) limits the steering change to 0.04 per 0.02 s, giving applied $u_s$. For longitudinal motion, the adapter uses the current $v_x$ and a fixed, identified **open-loop inverse map** to choose throttle $u_t$ or brake $u_b$. Both are constrained to $[0,0.6]$ and cannot be nonzero together. Its calibration is supplied and fixed; there is no acceleration feedback or online adaptation in this lower controller.

The achievable acceleration depends on speed and pedal limits. Denote the lowest and highest accelerations predicted by the fitted pedal model at speed $v_x$ by $a_{\min}(v_x)$ and $a_{\max}(v_x)$. At the **start** of control step $k$, the nominal effective longitudinal term is

$$
a_{x,k}^{\rm eff}=\operatorname{clip}\!\left(a_{x,k}^{\rm req},a_{\min}(v_{x,k}),a_{\max}(v_{x,k})\right).
$$

The adapter holds its selected pedals for the 0.02 s step. Since $v_x$ can change during that step, the model's instantaneous $a_x^{\rm model}(t)$ can vary slightly from $a_{x,k}^{\rm eff}$. The supplied [`contract.acceleration_bounds`](contract.py) computes the initial feasible interval and [`dynamics.predict`](dynamics.py) applies the same adapter when testing candidate actions. Within the fitted map's feasible interval, the pedal inverse is algebraically one-to-one at a fixed forward speed; residual PyChrono error is model-to-plant error, not a numerical failure to invert that map. Thus a model-based PI or rollout controller may optimize over $(u_s^{\rm req},a_x^{\rm req})$ without implementing the pedal inverse. The model's main lateral and yaw equations are

$$
\dot v_x=a_x^{\rm model}+r v_y-\frac{F_{yf}\sin\delta}{m},\qquad
\dot v_y=\frac{F_{yf}\cos\delta+F_{yr}}{m}-r v_x,
$$
$$
\dot r=\frac{l_fF_{yf}\cos\delta-l_rF_{yr}}{I_z},\qquad l_f+l_r=L.
$$

The front and rear slip-angle approximations are

$$
\alpha_f=\delta-\operatorname{atan2}(v_y+l_fr,\max(v_x,1)),\qquad
\alpha_r=-\operatorname{atan2}(v_y-l_rr,\max(v_x,1)).
$$

The lateral tire forces use $F_{yi}=F_{i,\mathrm{cap}}\tanh(C_i\alpha_i/F_{i,\mathrm{cap}})$. Their capacities depend on friction, static axle load, and a simplified sharing with longitudinal force. This is a smooth identified approximation, not an exact tire friction ellipse. The model's longitudinal term $a_x^{\rm model}$ is **not** exactly $\dot v_x$ in a turn: yaw/lateral velocity and front tire force also contribute in the first equation above. The precise fitted pedal map, including resistance and smooth saturation, is implemented in [`contract.py`](contract.py) and [`dynamics.py`](dynamics.py), with coefficients in [`model_parameters.json`](model_parameters.json). Students need not invert it to use the supplied high-level predictor. The measured steering state follows a fitted speed-dependent *linear* gain and first-order lag:

$$
\dot\delta=\frac{(s_g+s_vv_x^2)u_s-\delta}{\tau_s},
\qquad(s_g,s_v,\tau_s)=(0.551426,-0.00042031,0.007088\ {\rm s}).
$$

Below 2 m/s, the implementation blends the lateral/yaw equations with a kinematic bicycle response. [`dynamics.py`](dynamics.py) is the authoritative numerical definition, including all capacity floors, smoothing, and midpoint integration details. `dynamics.predict(state, command, previous_steering, scenario)` takes the full-mode `{"steering", "acceleration"}` request, applies the public adapter, and predicts one 0.02 s step. The lower-level `dynamics.step` and `dynamics.replay` instead take applied `[steering, throttle, brake]` inputs so that the archived validation traces remain reproducible. `dynamics.parameters()` intentionally rejects friction values other than 0.9 because the coefficients were not validated for another surface. The lower controller and predictor use the same nominal map; actual PyChrono acceleration can differ from $a_x^{\rm model}$, especially outside the measured driving range.

The longitudinal fit includes a high-speed correction beginning at 9.2 m/s. It was calibrated on separate straight-line drive and coast traces after the original quadratic drive fit underestimated acceleration near the 12 m/s course limit. This correction changes both the inverse adapter and predictor, while preserving their mutual consistency. In one independent fast slalom trace, 2 s fixed-applied-input forward-speed RMSE fell from 0.760 to 0.183 m/s and REF lateral RMSE from 0.181 to 0.103 m; this is a different run from the aggressive maneuver in the table below. The correction does not make the request a guaranteed physical acceleration: in a separate PyChrono straight-line check, a nominal $a_x^{\rm req}=0$ at approximately 10.5 m/s still increased speed by 0.207 m/s over 2.4 s. Keep a speed margin and check the closed-loop policy in PyChrono, especially near the 12 m/s failure boundary.

## Comparison with PyChrono

![PyChrono and provided-model fixed-input replay comparison](validation/model_comparison.png)

The published [validation traces and metrics](validation/metrics.json) compare the underlying vehicle model with PyChrono using the **same applied steering, throttle, and brake inputs**. They therefore measure prediction error given the actual applied inputs; they do not separately establish perfect tracking of an acceleration request by the new open-loop lower controller. Each prediction window starts at a measured state and runs open loop for 0.5, 1, or 2 s; intermediate measured states are not injected. Windows start 0.5 s apart and can overlap. The held-out mixed maneuver was excluded from coefficient fitting; the baseline course maneuver was included in the vehicle-coefficient fit. The first 0.8 s of the held-out trace is excluded from the window metrics. Steering simplification was separately fitted on calibration traces and checked on disjoint held-out driving traces, including the aggressive course maneuver summarized below.

| Dataset | Horizon | Forward-speed endpoint RMSE | REF lateral-position endpoint RMSE |
|---|---:|---:|---:|
| Held-out mixed maneuver | 0.5 s | 0.0371 m/s | 0.0042 m |
| Held-out mixed maneuver | 1.0 s | 0.0531 m/s | 0.0125 m |
| Held-out mixed maneuver | 2.0 s | 0.0832 m/s | 0.0382 m |
| Baseline course maneuver, included in fitting | 0.5 s | 0.0620 m/s | 0.0049 m |
| Baseline course maneuver, included in fitting | 1.0 s | 0.1204 m/s | 0.0135 m |
| Baseline course maneuver, included in fitting | 2.0 s | 0.2401 m/s | 0.0560 m |
| Separate held-out aggressive course maneuver (summary only) | 0.5 s | 0.2071 m/s | 0.0241 m |
| Separate held-out aggressive course maneuver (summary only) | 1.0 s | 0.3223 m/s | 0.0702 m |
| Separate held-out aggressive course maneuver (summary only) | 2.0 s | 0.4439 m/s | 0.2111 m |

The archived `validation/heldout_maneuver_mu0.9.npz` and `validation/course_baseline_maneuver.npz` contain `states` shaped `(N+1,7)`, `actions` shaped `(N,3)`, and 0.02 s samples. Run `validate_model.py` to regenerate the plot and metrics for those two public traces. The aggressive trace is summarized in the table but its controller and raw trajectory are not supplied. These are **short-window** tests. The high-speed longitudinal correction reduced the aggressive 2 s forward-speed error from 0.4925 to 0.4439 m/s but increased its REF lateral error from 0.1938 to 0.2111 m. Those errors are material when planned cone clearance is small. One open-loop replay of the entire race accumulates error, and accuracy at large sideslip, wheel spin/lock, altered friction, or extreme student policies is unverified. The model does not expose PyChrono's full roll/pitch, gear, engine, wheel, and tire state. Validate the final controller in PyChrono.
