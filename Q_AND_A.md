# Questions and answers

## Do I have to use both learning modes?

No. Submit an Easy plan, a Full controller, or both; if both, identify the result to evaluate. They share the PyChrono car, course, and success rules, but have different interfaces.

## Did the Full-mode acceleration input change Easy mode?

No. Easy still reads `easy_plan.json`; its supplied tracker converts plan nodes to steering and pedal inputs. Only Full uses the acceleration request and open-loop pedal adapter.

## Can I use a lower-level controller in Full mode and apply learning at a higher level?

Yes. A learned planner may choose targets for your path or speed tracker. Include both in your submitted `Controller`. Every 0.02 s, `act` must request steering and longitudinal acceleration; the adapter converts acceleration to throttle or brake. Explain each component's role in your report.

## Does requesting zero acceleration mean coasting?

No. Full's approximate inverse actuator map may apply positive throttle at zero requested acceleration to offset modeled resistance. It is open loop, so measured acceleration may differ; requests beyond modeled pedal capability saturate. See the [action interface](INTERFACE.md) and [vehicle model](MODEL.md).

## Is the Easy-mode DP plan guaranteed to finish safely in PyChrono?

No. Its virtual moves assume exact tracking and omit the full vehicle footprint. Check the plan with the local runner; a valid plan file can still fail in PyChrono.

## Can I use a reference path or speed in my controller?

Yes. Generate and track your own references. The slow example's internal path and speed only demonstrate the interface.

## May I use extra Python packages?

Yes for offline training; pin versions in `requirements.txt`. Evaluation uses the supplied Conda environment without installing extras. Include required learned weights.

## Which time determines performance?

Use `result.json`'s `finish_time_s`: interpolated simulation time when REF first crosses the finish after all eight gates. `last_cone_time_s` records gate eight; wall-clock controller latency is separate. Failure gets the minimum performance score.

## Can Easy mode enter the cone-spacing bonus?

No. The bonus evaluates a Full controller on basic and changed-spacing courses. Easy's `easy_plan.json` is a fixed move list for one course.

## Can I retrain for the bonus cone spacing?

No. Use the same Full controller code and learned object or weights on both courses. The unchanged controller or object may receive spacing as input. Bonus scenarios and scoring will be announced separately.

For installation or simulator errors, see [Troubleshooting](TROUBLESHOOTING.md).
