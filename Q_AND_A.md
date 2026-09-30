# Questions and answers

**Do I have to use both learning modes?**  
No. You may submit an Easy-mode plan, a Full-mode controller, or both. If you submit both, identify which result you want evaluated. The modes share the PyChrono vehicle, course, and success rules, but have different submission interfaces.

**Did the Full-mode acceleration input change Easy mode?**  
No. Easy mode still reads `easy_plan.json`. Its supplied tracker still turns plan nodes into native steering and pedal inputs. The acceleration request and its open-loop pedal adapter belong to Full mode only.

**Can I use a lower-level controller in Full mode and apply learning at a higher level?**  
Yes. You can design a lower-level path or speed tracker and use a learning-based planner to choose its targets. Both belong in your submitted `Controller`, whose `act` method must return a steering request and desired longitudinal acceleration every 0.02 s. The provided adapter converts that acceleration request to throttle or brake. Explain the roles of your learned and lower-level components in your report.

**Does requesting zero acceleration mean coasting?**  
No. The Full-mode adapter uses an approximate inverse actuator map, so a zero request can produce a positive throttle input to compensate for modeled resistance. Its map is open loop: measured PyChrono acceleration can differ from the request. Requests beyond the modeled pedal capability are saturated. See [the action interface](INTERFACE.md) and [vehicle model](MODEL.md).

**Is the Easy-mode DP plan guaranteed to finish safely in PyChrono?**  
No. The grid model assumes exact virtual moves and does not model the complete vehicle footprint or tracking error. Validate your plan using the local runner. A valid plan file is not automatically a successful scored run.

**Can I use a reference path or speed in my controller?**  
Yes. You may generate and track your own references. The supplied slow example has an internal path and speed only to demonstrate the interface; neither is prescribed for the challenge.

**Which time determines performance?**  
The finish time in `result.json` is the interpolated simulation time when REF first crosses the finish after all eight gates. `last_cone_time_s` is the eighth gate crossing, and wall-clock controller latency is a separate diagnostic. A failed run receives the minimum performance score.

**Can I retrain for the bonus cone spacing?**  
No. The same controller code and learned object or weights must be used for the basic and bonus courses. The changed spacing may be supplied as an input to that unchanged controller or learned object. The detailed bonus scenarios and scoring will be announced separately.

For an installation or simulator failure, see [Troubleshooting](TROUBLESHOOTING.md).
