"""Reproduce published comparisons; requires numpy and matplotlib."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from dynamics import replay, parameters


def main():
    evidence = Path(__file__).resolve().parent / 'validation'
    fig, axs = plt.subplots(3, 2, figsize=(12, 9), constrained_layout=True)
    metrics = []
    cases = [('heldout_maneuver_mu0.9.npz', 'Held-out mixed maneuver', 40),
             ('course_baseline_maneuver.npz', '15 m / 8-cone baseline pilot', 0)]
    for col, (filename, label, start) in enumerate(cases):
        data = np.load(evidence / filename)
        s, u, p = data['states'], data['actions'], parameters()
        snapshot = 400 if col == 0 else 600
        pred = replay(s[snapshot], u[snapshot:snapshot+100], .9, p)
        truth = s[snapshot:snapshot+101]
        t = np.arange(len(pred)) * .02
        axs[0, col].plot(t, truth[:, 3], label='PyChrono')
        axs[0, col].plot(t, pred[:, 3], '--', label='Control-oriented vehicle model')
        axs[0, col].set(title=label + ' (2 s open-loop example)', xlabel='Horizon [s]', ylabel='Forward speed [m/s]')
        axs[0, col].legend()
        axs[1, col].plot(truth[:, 0], truth[:, 1])
        axs[1, col].plot(pred[:, 0], pred[:, 1], '--')
        axs[1, col].set(xlabel='Global X [m]', ylabel='Global Y [m]')
        rows = []
        for horizon in [.5, 1., 2.]:
            h = round(horizon / .02)
            errors = [replay(s[k], u[k:k+h], .9, p)[-1] - s[k+h]
                      for k in range(start, len(u)-h+1, 25)]
            rmse = np.sqrt(np.mean(np.array(errors)**2, axis=0))
            rows.append(dict(horizon_s=horizon, windows=len(errors), rmse=rmse.tolist()))
        for j, key in [(3, 'vx [m/s]'), (1, 'Y [m]')]:
            axs[2, col].plot([x['horizon_s'] for x in rows],
                             [x['rmse'][j] for x in rows], 'o-', label=key)
        axs[2, col].set(xlabel='Prediction horizon [s]', ylabel='RMSE (units in legend)')
        axs[2, col].legend()
        metrics.append(dict(dataset=filename, metrics=rows))
    for ax in axs.flat:
        ax.grid(alpha=.2)
    fig.savefig(evidence / 'model_comparison.png', dpi=150)
    plt.close(fig)
    (evidence / 'metrics.json').write_text(json.dumps(dict(
        state_columns=['X', 'Y', 'psi', 'vx', 'vy', 'r', 'delta'], datasets=metrics),
        indent=2), encoding='utf-8')
    print('Saved validation/model_comparison.png and metrics.json')


if __name__ == '__main__':
    main()
