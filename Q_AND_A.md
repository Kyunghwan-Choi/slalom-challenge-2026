# Questions and answers

## Do I have to use both learning modes?

No. Submit an Easy plan, a Full controller, or both; if both, identify the result to evaluate. They share the PyChrono car, course, and success rules, but have different interfaces.

## Can I use a lower-level controller in Full mode and apply learning at a higher level?

Yes. A learned planner may choose targets for your path or speed tracking controller. Include both in your submitted `Controller`. Every 0.02 s, `Controller.act` must request steering and longitudinal acceleration; the runner converts acceleration to throttle or brake. Explain each component's role in your report.

## Is the Easy-mode DP plan guaranteed to finish safely in PyChrono?

No. Its simplified grid moves assume exact tracking and omit the full vehicle footprint. Check the plan with the local runner; a valid plan file can still fail in PyChrono.

## Can I use a reference path or speed in my controller?

Yes. Generate and track your own references. The 5 m/s example's path and speed only demonstrate the interface.

## May I use extra Python packages?

Yes for offline training; pin versions in `requirements.txt`. Evaluation uses the supplied Conda environment without installing extras. Include required learned weights.

For installation or simulator errors, see [Troubleshooting](TROUBLESHOOTING.md).
