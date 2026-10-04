"""Provided reduced predictor, calibrated at mu=0.9; not the grading plant.
State: X_REF,Y_REF,psi,vx_COM,vy_COM,r,mean_front_steer. SI units.
Internal input: normalized steering, throttle, brake. See MODEL.md.
"""

import json
from functools import lru_cache
from pathlib import Path

import numpy as np

try:
    from numba import njit
except ImportError:

    def njit(*args, **kwargs):
        return lambda function: function


MASS = 1909.882955108
L = 2.776
REF_OFFSET = 1.371


@njit(cache=True)
def derivative(s, u, mu, p):
    # Public coefficients describe only the calibrated mu=0.9 regime.
    X, Y, psi, vx, vy, r, delta = s
    (
        cf,
        cr,
        iz,
        lf,
        ms,
        sg,
        sv,
        st,
        a0,
        av,
        av2,
        d0,
        d2,
        bg,
        tc,
        bc,
        exponent,
        brake_exponent,
        high_floor,
        drag_relief,
        ramp_start,
        ramp_end,
    ) = p
    lr = L - lf
    speed = max(vx, 0.0)
    drivecap = max(0.1, mu * 9.81 * tc)
    brakecap = max(0.1, mu * 9.81 * bc)
    drivegain = max(
        0.1, a0 + av * speed + av2 * speed * speed, high_floor if speed >= ramp_start else 0.1
    )
    drive = drivecap * np.tanh(drivegain * max(u[1], 0.0) ** exponent / drivecap)
    braking = (
        brakecap * np.tanh(bg * max(u[2], 0.0) ** brake_exponent / brakecap) * np.tanh(vx / 0.2)
    )
    ramp_fraction = min(1.0, max(0.0, (speed - ramp_start) / (ramp_end - ramp_start)))
    ramp = ramp_fraction * ramp_fraction * (3.0 - 2.0 * ramp_fraction)
    ax = drive - braking - d0 * np.tanh(vx / 0.2) - d2 * vx * abs(vx) + drag_relief * ramp
    fzf = MASS * 9.81 * lr / L
    fzr = MASS * 9.81 * lf / L
    fx = MASS * ax
    fxfront = 0.6 * fx if fx < 0 else 0.0
    fxrear = fx - fxfront
    capf = mu * ms * fzf * np.sqrt(max(0.04, 1 - (fxfront / max(mu * ms * fzf, 1.0)) ** 2))
    capr = mu * ms * fzr * np.sqrt(max(0.04, 1 - (fxrear / max(mu * ms * fzr, 1.0)) ** 2))
    af = delta - np.arctan2(vy + lf * r, max(vx, 1.0))
    ar = -np.arctan2(vy - lr * r, max(vx, 1.0))
    fyf = capf * np.tanh(cf * af / capf)
    fyr = capr * np.tanh(cr * ar / capr)
    dvy = (fyf * np.cos(delta) + fyr) / MASS - r * vx
    dr = (lf * fyf * np.cos(delta) - lr * fyr) / iz
    blend = min(1.0, max(0.0, vx / 2.0))
    rkin = vx * np.tan(delta) / L
    dvy = blend * dvy + (1 - blend) * (lr * rkin - vy) / 0.12
    dr = blend * dr + (1 - blend) * (rkin - r) / 0.12
    target = (sg + sv * vx * vx) * u[0]
    refvy = vy + REF_OFFSET * r
    return np.array(
        [
            vx * np.cos(psi) - refvy * np.sin(psi),
            vx * np.sin(psi) + refvy * np.cos(psi),
            r,
            ax + r * vy - fyf * np.sin(delta) / MASS,
            dvy,
            dr,
            (target - delta) / st,
        ]
    )


@njit(cache=True)
def step(s, u, mu, p, dt=0.02):
    z = s.copy()
    n = max(1, int(np.ceil(dt / 0.004)))
    h = dt / n
    for _ in range(n):
        k = derivative(z, u, mu, p)
        z += h * derivative(z + 0.5 * h * k, u, mu, p)
    return z


@njit(cache=True)
def replay(s, actions, mu, p):
    rows = np.empty((len(actions) + 1, 7))
    rows[0] = s
    for i in range(len(actions)):
        rows[i + 1] = step(rows[i], actions[i], mu, p)
    return rows


@lru_cache(maxsize=1)
def _nominal_parameters():
    return np.array(
        json.loads(Path(__file__).with_name("model_parameters.json").read_text(encoding="utf-8"))[
            "parameters"
        ]
    )


def parameters(mu=0.9):
    if abs(mu - 0.9) > 1e-9:
        raise ValueError("Public predictor is calibrated only at mu=0.9")
    return _nominal_parameters().copy()


def predict(state, command, previous_steering, config):
    from contract import apply_action

    applied = np.array(apply_action(command, previous_steering, config, float(state[3])))
    nxt = step(
        np.array(state, dtype=float),
        applied,
        config["friction"],
        parameters(config["friction"]),
        config["control_dt_s"],
    )
    return nxt, applied
