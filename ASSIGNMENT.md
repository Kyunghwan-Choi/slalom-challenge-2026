# Slalom Challenge: assignment and evaluation

Use course methods to take the provided PyChrono car through eight alternating slalom gates and reach the finish line as quickly as possible. In Easy mode, you design a plan for the supplied tracker; in Full mode, you design the vehicle controller. No reference path or reference speed is prescribed. The starter controller tracks its own slow path only to demonstrate the software interface.

This is an **individual** assignment for the 2026 *Learning-Based Control for Mobility Systems* course. Your report and implementation must show how your design uses course concepts such as problem formulation, DP/VI/PI, value or policy approximation, rollout, or multistep lookahead. You do not need to use every method. A generic RL package applied without a course-based controller design and explanation receives zero credit.

| Milestone | Korea Standard Time (UTC+09:00) |
|---|---|
| Official course announcement | October 1, 2026, 13:00 |
| Submission deadline | November 1, 2026, 23:59 |
| Race results and individual questions in class | November 3, 2026; about five minutes per student |

Email your submission and questions to [fairytale@kaist.ac.kr](mailto:fairytale@kaist.ac.kr). The [submission guide](SUBMISSION_AND_AI.md) gives the minimal file list and report template.

## Course and vehicle

The course uses a fixed **global $(X,Y)$ frame**: +X points toward the finish, and +Y is left when facing +X. **REF** is the vehicle model's reference point, with global position $p_{\rm REF}=(X_{\rm REF},Y_{\rm REF})$. **COM** is the vehicle's center of mass, with global position $p_{\rm COM}$. The vehicle's body $x$ axis points forward and body $y$ axis points left. Its heading $\psi$ is the counterclockwise angle from global +X to body-forward. REF lies 1.371 m ahead of COM along the body axis:

$$
p_{\rm REF}=p_{\rm COM}+1.371(\cos\psi,\sin\psi)\ {\rm m}.
$$

REF is a point defined by the PyChrono vehicle model; it was not introduced solely to check gates or collisions. The drawings show the course and the two vehicle points. The [vehicle model](MODEL.md) gives further detail.

![Top view of the eight-cone course, alternating gates, road boundary, and finish line](assets/course_layout.svg)

![Global and body coordinates, heading, REF, COM, and the vehicle footprint](assets/ref_com.svg)

The basic course is level rigid terrain with road friction coefficient $\mu=0.9$. The plant is PyChrono 10.0.0's BMW E90 with TMeasy tires. Its nominal initial REF position is $(X_{\rm REF},Y_{\rm REF})=(0,0)$ with $\psi=0$. It starts from the common state produced by a 0.8 s settling procedure; use the **first observation**, rather than assuming its measured speed and position are exactly zero. The road extends from $Y=-5.5$ to $Y=5.5$ m.

Eight fixed cones of radius 0.25 m have centers at $(20+15j,0)$ m for $j=0,\ldots,7$. Pass the first cone on its +Y side, the next on its −Y side, and continue alternating. The finish line is at $X_{\rm REF}=145$ m, 20 m after the last cone. A 60 s simulation-time limit applies. The controller runs every 0.02 s and the vehicle physics every 0.001 s.

Two interfaces use **the same plant and success rules**. In `easy` mode you submit a discrete position-and-speed plan; the supplied lower controller tracks it. In `full` mode your controller receives the vehicle observation and requests steering and longitudinal acceleration every 0.02 s. A supplied open-loop adapter turns the requested acceleration into PyChrono throttle or brake. Either mode can be submitted. The easy mode is intended as a first DP exercise and has a more restrictive planning model; see [running and learning modes](RUNNING.md).
In Full mode, you may add your own lower-level tracking controller and apply learning-based methods to a higher-level planner; your submitted controller must still produce the published steering and acceleration action. The Full-mode action change does not alter Easy mode's supplied tracker or plan interface. [Questions and answers](Q_AND_A.md) covers this distinction.

### Gate and safety rules

REF determines gate crossings and finish time. A gate is checked when REF first crosses that cone's X plane in the +X direction. Linear interpolation within the physics step gives its lateral crossing position. The crossing is valid when REF is **on the designated side** of the cone center: $\sigma_j(Y_{\rm REF}(t_j)-Y_j)>0$, where $\sigma_j=+1$ for a +Y pass and $-1$ for a −Y pass. There is **no fixed lateral gate-distance requirement**. All eight gates must be passed in order. The side rule enforces the alternating slalom; cone contact and road containment are checked separately.

Cone contact fails the run. The evaluator also checks a conservative, body-aligned car footprint: a 4.7 m × 1.9 m rectangle centered at COM. Overlap with a cone's 0.25 m-radius footprint fails even without a reported physical contact. The complete footprint must remain within the road. A wrong-side gate crossing, road departure, invalid action, controller exception, exceeding the vehicle limits, or reaching the 60 s limit before finishing also fails. The vehicle limits are COM forward speed $-1\le v_x\le12$ m/s, global REF X progress speed $\dot X_{\rm REF}\le12$ m/s, and heading $|\psi|\le1.3$ rad. [`course.json`](course.json) gives the machine-readable values. A failure event has priority if it occurs in the same physics step as a finish crossing.

Easy-mode DP plans REF positions so its planned gate crossings use the same coordinate as the evaluator; the actual car footprint is still tested by PyChrono. The [input and output interface](INTERFACE.md) identifies which measurements are at REF and which are at COM.

## Mathematical objective

The task is a constrained free-final-time control problem. Let $\xi$ denote the full vehicle state, $u$ the controller command, $\mathcal C$ the course, and $t_j$ the forward crossing time of cone $j$'s X plane. A compact statement is

$$
\begin{aligned}
\min_{u(\cdot),\,t_f}\quad &t_f\\
\text{s.t.}\quad
&\xi(0)=\xi_{\rm settled},\qquad
  \dot\xi(t)=F_{\rm vehicle}(\xi(t),u(t);\mu,\mathcal C),\\
&u(t)\in\mathcal U,\quad
  -1\le v_x(t)\le12\ {\rm m/s},\quad
  \dot X_{\rm REF}(t)\le12\ {\rm m/s},\quad
  |\psi(t)|\le1.3\ {\rm rad},\\
&\text{the complete vehicle footprint stays on the road and touches no cone for }0\le t\le t_f,\\
&0<t_0<t_1<\cdots<t_7<t_f,\\
&X_{\rm REF}(t_j)=X_j,\quad
  \dot X_{\rm REF}(t_j)>0,\quad
  \sigma_j\bigl(Y_{\rm REF}(t_j)-Y_j\bigr)>0
  \quad(j=0,\ldots,7),\\
&X_{\rm REF}(t_f)=X_{\rm finish},\quad
  \dot X_{\rm REF}(t_f)>0,\quad
  t_f\le60\ {\rm s}.
\end{aligned}
$$

Here $(X_j,Y_j)$ is cone $j$'s global center and $\sigma_j=+1$ for an assigned +Y pass or $-1$ for an assigned −Y pass. The full-mode command is $u=(u_s^{\rm req},a_x^{\rm req})$, a steering request and a desired longitudinal acceleration, with $\mathcal U=[-0.8,0.8]\times[-7,7\,\mathrm{m/s^2}]$. The supplied adapter converts this request to vehicle inputs, subject to speed-dependent actuator saturation. The constraints describe the evaluation task; [`course.json`](course.json), [the interface](INTERFACE.md), and the local runner specify the discrete command limits and event checks exactly. The PyChrono simulation is the evaluation plant. The supplied control-oriented model is an approximation that you may use in your design.

## What is supplied and what you submit

This repository supplies the PyChrono practice runner, course definition, controller contract, two mode interfaces, a slow demonstration controller, an approximate vehicle model with validation data, and a report template. The practice runner records `result.json` and `trajectory.npz` so that you can validate your work locally.

You submit **one ZIP containing your PDF report and runnable solution**. The solution includes the mode-specific entry (`easy_plan.json` or `controller.py`) and any helper, training, or learned-weight files needed to reproduce it. You may submit both modes; indicate which result you want evaluated. The report explains your design and your own validation. See [submission and AI use](SUBMISSION_AND_AI.md) for details.

## Assessment

- **Basic-course performance:** A successful run is scored by finish time against absolute-time thresholds, not by class rank. Numerical thresholds and point allocation will be announced separately.
- **Failure:** A failed run receives the minimum performance score.
- **Course-based design and understanding:** The code, report, and November 3 individual discussion are used to assess your method and your ability to explain it. Report length itself is not graded.
- **Full-mode bonus:** A changed-cone-spacing track can add points to a Full-mode basic score. Easy mode's fixed plan file is not a bonus entry. The detailed scenarios and bonus cap will be announced with the scoring table. Use the **same controller code, design, and learned object or weights** as on the basic course. Only the cone-spacing parameter may change; the unchanged controller or learned object may use it as an input. Retraining, replacing weights, or switching controllers is not allowed. Starting-pose variation is not a bonus task.

If a simulator or scoring issue appears, use the report checklist in [Troubleshooting](TROUBLESHOOTING.md). Confirmed issues and workarounds will be posted there.
