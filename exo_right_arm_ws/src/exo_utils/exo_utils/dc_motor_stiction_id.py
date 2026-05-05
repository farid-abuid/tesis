#!/usr/bin/env python3
"""
Stribeck friction parameter identification over Teensy serial.

Strategy
--------
Instead of a noisy ramp-until-motion approach, this script uses a
**velocity-controlled steady-state sweep**:

  1. Velocity sweep (bidirectional)
     A sequence of target speeds (from near-zero to max) is commanded via
     a closed-loop PD velocity controller running on the host. At each
     speed the motor is held for `--dwell-s` seconds; the last
     `--dwell-avg-s` seconds of torque are averaged → one (|ω|, τ_f) pair.
     Doing this in both directions and averaging eliminates offset/gravity.

  2. Sine excitation (optional, for inertia J)
     A sinusoidal torque sweep identical to the original script estimates J
     with a linear regression. J does NOT appear in the Stribeck model but
     is needed for full dynamics simulation.

  3. Curve fit
     scipy.optimize.curve_fit fits
         τ_f = τ_c·sign(ω) + b·ω + (τ_s − τ_c)·exp(−(|ω|/v_s)²)
     to the (ω, τ_f) pairs collected in step 1.

Why this beats the ramp method
-------------------------------
• No threshold crossing → no consecutive-sample noise.
• Many repeated measurements per speed → averaging kills noise.
• Bidirectional sweep cancels constant offsets.
• scipy curve_fit with a good initial guess is robust to the
  nonlinear Gaussian term.

Usage example
-------------
python stribeck_id.py --motor-id 1 \\
    --speeds 0.05 0.1 0.2 0.4 0.7 1.0 1.5 2.0 3.0 4.0 5.0 \\
    --dwell-s 2.0 --dwell-avg-s 0.8 \\
    --kp 0.4 --kd 0.05 \\
    --sine-amplitude 1.8 --sine-duration-s 20
"""

import argparse
import csv
import math
import struct
import time
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import scipy.optimize as opt
import serial

# ---------------------------------------------------------------------------
# Serial protocol (identical to original)
# ---------------------------------------------------------------------------
HEADER1 = 0xAA
HEADER2 = 0x55
CMD_TORQUE = 1
STATUS_STRUCT = struct.Struct("<Bfff")


@dataclass
class MotorStatus:
    motor_id: int
    tau_meas: float
    dq: float
    q: float


class SerialMotorInterface:
    """Implements the Teensy frame format from teensy_hardware.ino."""

    def __init__(self, port: str, baudrate: int, timeout: float, motor_id: int, retries: int = 3):
        self.motor_id = motor_id
        self.serial = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        self.retries = max(1, retries)

    @staticmethod
    def _checksum(payload: bytes) -> int:
        cs = 0
        for b in payload:
            cs ^= b
        return cs

    def _build_command(self, cmd_type: int, value: float) -> bytes:
        meta = ((cmd_type & 0x0F) << 4) | 0x01
        payload = struct.pack("<BBf", meta, self.motor_id, float(value))
        return bytes([HEADER1, HEADER2]) + payload + bytes([self._checksum(payload)])

    def _read_exact(self, n: int) -> bytes:
        data = self.serial.read(n)
        if len(data) != n:
            raise TimeoutError(f"Expected {n} bytes, got {len(data)}")
        return data

    def _read_status_frame(self) -> MotorStatus:
        while True:
            if self._read_exact(1)[0] != HEADER1:
                continue
            if self._read_exact(1)[0] != HEADER2:
                continue
            raw = self._read_exact(STATUS_STRUCT.size)
            mid, tau, dq, q = STATUS_STRUCT.unpack(raw)
            return MotorStatus(motor_id=mid, tau_meas=tau, dq=dq, q=q)

    def command_torque_and_read(self, tau_cmd: float) -> MotorStatus:
        frame = self._build_command(CMD_TORQUE, tau_cmd)
        last_err = None
        for _ in range(self.retries):
            self.serial.write(frame)
            try:
                st = self._read_status_frame()
                if st.motor_id != self.motor_id:
                    raise TimeoutError(f"Wrong motor_id {st.motor_id}")
                return st
            except TimeoutError as e:
                last_err = e
                self.serial.reset_input_buffer()
        raise TimeoutError(f"Serial failed after {self.retries} retries: {last_err}")

    def stop_motor(self, hold_s: float = 0.3) -> None:
        try:
            self.command_torque_and_read(0.0)
        except TimeoutError:
            pass
        time.sleep(hold_s)

    def close(self) -> None:
        if self.serial and self.serial.is_open:
            self.serial.close()


# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------

def low_pass_filter(sig: np.ndarray, dt: float, cutoff_hz: float) -> np.ndarray:
    if cutoff_hz <= 0:
        return sig.copy()
    rc = 1.0 / (2.0 * math.pi * cutoff_hz)
    alpha = dt / (rc + dt)
    out = np.empty_like(sig)
    out[0] = sig[0]
    for i in range(1, sig.size):
        out[i] = out[i - 1] + alpha * (sig[i] - out[i - 1])
    return out


def write_csv(path: Path, header: list[str], columns: list) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(header)
        for row in zip(*columns):
            w.writerow([f"{float(v):.9f}" for v in row])


# ---------------------------------------------------------------------------
# Stribeck model
# ---------------------------------------------------------------------------

def stribeck_torque(omega: np.ndarray, tau_c: float, b: float, tau_s: float, v_s: float) -> np.ndarray:
    """
    τ_f = τ_c·sign(ω) + b·ω + (τ_s − τ_c)·exp(−(|ω|/v_s)²)

    Note: this function maps signed ω → signed τ_f,  suitable for
    regression against measured torque (sign already encodes direction).
    """
    return (
        tau_c * np.sign(omega)
        + b * omega
        + (tau_s - tau_c) * np.exp(-((np.abs(omega) / v_s) ** 2)) * np.sign(omega)
    )


def stribeck_torque_magnitude(speed: np.ndarray, tau_c: float, b: float, tau_s: float, v_s: float) -> np.ndarray:
    """Same model but for |ω| ≥ 0 → τ_f ≥ 0  (used for the steady-state sweep fit)."""
    return tau_c + b * speed + (tau_s - tau_c) * np.exp(-((speed / v_s) ** 2))


# ---------------------------------------------------------------------------
# Phase 1 – Bidirectional steady-state velocity sweep
# ---------------------------------------------------------------------------

def run_velocity_sweep(interface: SerialMotorInterface, args: argparse.Namespace) -> dict:
    """
    For each target speed in args.speeds, close a PD velocity loop and
    hold the motor at that speed.  After settling, average torque over
    the last `dwell_avg_s` seconds.  Repeat for both directions.

    Returns arrays of (signed_omega, tau_f) ready for curve fitting.
    """
    speeds = np.asarray(args.speeds, dtype=float)
    signed_speeds = np.concatenate([speeds, -speeds])  # both directions

    omega_list: list[float] = []
    tau_list: list[float] = []

    n_dwell = int(args.dwell_s / args.dt)
    n_avg   = int(args.dwell_avg_s / args.dt)
    n_avg   = min(n_avg, n_dwell)

    for idx, target_omega in enumerate(signed_speeds):
        print(f"  Speed {idx + 1}/{len(signed_speeds)}: target ω = {target_omega:+.3f} rad/s")

        tau_buf: list[float] = []
        omega_buf: list[float] = []

        # Integral windup guard
        integrator = 0.0
        prev_err = 0.0

        for step in range(n_dwell):
            status = interface.command_torque_and_read(0.0)  # read first, then command
            err = target_omega - status.dq

            # PD (no integrator to keep it simple and avoid windup)
            tau_cmd = args.kp * err + args.kd * (err - prev_err) / args.dt
            tau_cmd = float(np.clip(tau_cmd, -args.max_current, args.max_current))
            prev_err = err

            # re-issue the actual command
            status = interface.command_torque_and_read(tau_cmd)

            if step >= (n_dwell - n_avg):
                tau_buf.append(status.tau_meas)
                omega_buf.append(status.dq)

            time.sleep(args.dt)

        interface.stop_motor(hold_s=args.settle_s)

        if len(tau_buf) == 0:
            continue

        tau_avg   = float(np.mean(tau_buf))
        omega_avg = float(np.mean(omega_buf))

        # Sanity: make sure we actually moved
        if abs(omega_avg) < args.motion_vel_threshold:
            print(f"    WARNING: motor barely moved (ω_avg={omega_avg:.4f}), skipping point.")
            continue

        omega_list.append(omega_avg)
        tau_list.append(tau_avg)
        print(f"    ω_avg={omega_avg:+.4f} rad/s  τ_avg={tau_avg:+.4f} Nm")

    return {
        "omega": np.asarray(omega_list, dtype=float),
        "tau":   np.asarray(tau_list,   dtype=float),
    }


# ---------------------------------------------------------------------------
# Phase 2 – Sine excitation for inertia J (optional but recommended)
# ---------------------------------------------------------------------------

def run_sine_excitation(interface: SerialMotorInterface, args: argparse.Namespace) -> dict:
    omega_exc = 2.0 * math.pi * args.sine_frequency
    n = int(args.sine_duration_s / args.dt)
    t0 = time.monotonic()

    t_arr        = np.zeros(n)
    tau_cmd_arr  = np.zeros(n)
    tau_meas_arr = np.zeros(n)
    q_arr        = np.zeros(n)
    dq_arr       = np.zeros(n)

    for k in range(n):
        tk = time.monotonic() - t0
        tau_cmd = args.sine_amplitude * math.sin(omega_exc * tk)
        st = interface.command_torque_and_read(tau_cmd)
        t_arr[k]       = tk
        tau_cmd_arr[k] = tau_cmd
        tau_meas_arr[k]= st.tau_meas
        q_arr[k]       = st.q
        dq_arr[k]      = st.dq
        time.sleep(args.dt)

    interface.stop_motor(hold_s=0.3)
    n_discard = 300
    return {
        "t":        t_arr[n_discard:-n_discard],
        "tau_cmd":  tau_cmd_arr[n_discard:-n_discard],
        "tau_meas": tau_meas_arr[n_discard:-n_discard],
        "q":        q_arr[n_discard:-n_discard],
        "dq":       dq_arr[n_discard:-n_discard],
    }


def estimate_inertia(data: dict, args: argparse.Namespace) -> dict:
    """Least-squares estimate of J using τ = J·α + τ_f(ω)."""
    t  = data["t"]
    dq = data["dq"]
    tau_meas = data["tau_meas"]
    dt_mean  = float(np.mean(np.diff(t)))

    dq_f  = low_pass_filter(dq, dt_mean, args.velocity_cutoff_hz)
    ddq   = np.gradient(dq_f, t)

    mask  = np.abs(dq_f) > args.regression_min_speed
    X     = ddq[mask].reshape(-1, 1)
    y_res = tau_meas[mask] - stribeck_torque(
        dq_f[mask],
        args._tau_c, args._b, args._tau_s, args._v_s
    )
    J     = float(np.linalg.lstsq(X, y_res, rcond=None)[0][0])

    tau_pred = J * ddq + stribeck_torque(dq_f, args._tau_c, args._b, args._tau_s, args._v_s)
    residual = tau_meas - tau_pred
    ss_res   = float(np.sum(residual ** 2))
    ss_tot   = float(np.sum((tau_meas - np.mean(tau_meas)) ** 2))
    r2       = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else float("nan")
    rmse     = float(np.sqrt(np.mean(residual ** 2)))

    return {
        "J": J, "r2": r2, "rmse": rmse,
        "tau_pred": tau_pred,
        "dq_filt": dq_f, "ddq": ddq,
        "t": t, "tau_meas": tau_meas, "tau_cmd": data["tau_cmd"],
        "dq_raw": dq,
    }


# ---------------------------------------------------------------------------
# Phase 3 – Stribeck curve fit
# ---------------------------------------------------------------------------

def fit_stribeck(sweep: dict, args: argparse.Namespace) -> dict:
    """
    Fit τ_f = τ_c·sign(ω) + b·ω + (τ_s − τ_c)·exp(−(|ω|/v_s)²)
    to steady-state (ω, τ) sweep data.

    Uses the MAGNITUDE form for curve_fit, then checks sign consistency.
    """
    omega = sweep["omega"]
    tau   = sweep["tau"]

    # Positive-side data (friction magnitude vs speed)
    pos_mask   = omega > 0
    neg_mask   = omega < 0
    speeds_pos = np.abs(omega[pos_mask])
    tau_pos    = np.abs(tau[pos_mask])
    speeds_neg = np.abs(omega[neg_mask])
    tau_neg    = np.abs(tau[neg_mask])

    # Average positive and negative for each speed (if matched)
    speeds_all = np.concatenate([speeds_pos, speeds_neg])
    tau_all    = np.concatenate([tau_pos,    tau_neg])

    # Initial guess: τ_c from low-speed torque, b from high-speed slope
    sort_idx = np.argsort(speeds_all)
    sp_s     = speeds_all[sort_idx]
    ta_s     = tau_all[sort_idx]

    tau_c0 = float(np.median(ta_s[:max(1, len(ta_s) // 6)]))
    tau_s0 = float(np.max(ta_s[:max(1, len(ta_s) // 3)])) * 1.1
    tau_s0 = max(tau_s0, tau_c0 * 1.05)
    b0     = float(max(0.0, (ta_s[-1] - ta_s[0]) / (sp_s[-1] - sp_s[0] + 1e-6)))
    v_s0   = float(np.median(sp_s)) * 0.3

    p0     = [tau_c0, b0, tau_s0, v_s0]
    bounds = ([0, 0, 0, 1e-4], [np.inf, np.inf, np.inf, np.inf])

    try:
        popt, pcov = opt.curve_fit(
            stribeck_torque_magnitude,
            speeds_all, tau_all,
            p0=p0, bounds=bounds,
            maxfev=10000,
        )
    except RuntimeError as e:
        print(f"  WARNING: curve_fit did not converge: {e}")
        popt = np.asarray(p0)
        pcov = np.full((4, 4), np.nan)

    tau_c_fit, b_fit, tau_s_fit, v_s_fit = popt
    perr = np.sqrt(np.diag(pcov)) if not np.any(np.isnan(pcov)) else [np.nan] * 4

    # Goodness of fit on magnitude data
    tau_pred_mag = stribeck_torque_magnitude(speeds_all, *popt)
    ss_res = float(np.sum((tau_all - tau_pred_mag) ** 2))
    ss_tot = float(np.sum((tau_all - np.mean(tau_all)) ** 2))
    r2     = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else float("nan")
    rmse   = float(np.sqrt(np.mean((tau_all - tau_pred_mag) ** 2)))

    return {
        "tau_c":  float(tau_c_fit),
        "b":      float(b_fit),
        "tau_s":  float(tau_s_fit),
        "v_s":    float(v_s_fit),
        "tau_c_std": float(perr[0]),
        "b_std":     float(perr[1]),
        "tau_s_std": float(perr[2]),
        "v_s_std":   float(perr[3]),
        "r2":    r2,
        "rmse":  rmse,
        "speeds_fit": speeds_all,
        "tau_fit_data": tau_all,
    }


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_stribeck_curve(sweep: dict, stribeck_params: dict, run_dir: Path) -> None:
    omega = sweep["omega"]
    tau   = sweep["tau"]
    tc    = stribeck_params["tau_c"]
    b     = stribeck_params["b"]
    ts    = stribeck_params["tau_s"]
    vs    = stribeck_params["v_s"]

    speed_range = np.linspace(0, np.max(np.abs(omega)) * 1.1, 400)
    tau_curve   = stribeck_torque_magnitude(speed_range, tc, b, ts, vs)

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.scatter(np.abs(omega), np.abs(tau), color="steelblue", s=60,
               zorder=5, label="Steady-state measurements")
    ax.plot(speed_range, tau_curve, color="tomato", linewidth=2,
            label=(f"Stribeck fit\n"
                   f"τ_c={tc:.4f}  b={b:.4f}\n"
                   f"τ_s={ts:.4f}  v_s={vs:.4f}"))
    ax.set_xlabel("Speed |ω| [rad/s]")
    ax.set_ylabel("Friction torque |τ_f| [Nm]")
    ax.set_title(f"Stribeck Curve  (R²={stribeck_params['r2']:.4f})")
    ax.legend()
    ax.grid(True)
    plt.tight_layout()
    plt.savefig(run_dir / "stribeck_curve.png", dpi=150)
    plt.show()


def plot_sine_fit(proc: dict, run_dir: Path) -> None:
    t = proc["t"]
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    axes[0].plot(t, proc["tau_cmd"],  label="τ_cmd",  linewidth=1)
    axes[0].plot(t, proc["tau_meas"], label="τ_meas", linewidth=1)
    axes[0].plot(t, proc["tau_pred"], label="τ_pred", linewidth=1, linestyle="--")
    axes[0].set_ylabel("Torque [Nm]")
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(t, proc["dq_raw"],  label="dq_raw",  linewidth=1, alpha=0.6)
    axes[1].plot(t, proc["dq_filt"], label="dq_filt", linewidth=1)
    axes[1].set_ylabel("Velocity [rad/s]")
    axes[1].legend()
    axes[1].grid(True)

    axes[2].plot(t, proc["ddq"], label="ddq", linewidth=1)
    axes[2].set_ylabel("Accel [rad/s²]")
    axes[2].set_xlabel("Time [s]")
    axes[2].legend()
    axes[2].grid(True)

    plt.tight_layout()
    plt.savefig(run_dir / "sine_fit.png", dpi=150)
    plt.show()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stribeck friction + inertia identification for DC motor via Teensy"
    )
    # Serial
    parser.add_argument("--port",      type=str, default="/dev/teensy_motor")
    parser.add_argument("--baudrate",  type=int, default=460800)
    parser.add_argument("--motor-id",  type=int, required=True)
    parser.add_argument("--dt",        type=float, default=0.01,
                        help="Control loop period [s]")
    parser.add_argument("--max-current", type=float, default=5.0,
                        help="Torque/current hard limit [Nm or A]")
    parser.add_argument("--output-dir", type=str, default="stribeck_id_runs")

    # Velocity sweep
    parser.add_argument("--speeds", type=float, nargs="+",
                        default=[0.05, 0.1, 0.2, 0.35, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0],
                        help="Target speeds for steady-state sweep [rad/s]")
    parser.add_argument("--dwell-s",     type=float, default=2.5,
                        help="Total dwell time per speed point [s]")
    parser.add_argument("--dwell-avg-s", type=float, default=1.0,
                        help="Averaging window at end of dwell [s]")
    parser.add_argument("--settle-s",    type=float, default=1.5,
                        help="Coast-down time between speed steps [s]")
    parser.add_argument("--kp", type=float, default=0.5,
                        help="PD velocity loop proportional gain")
    parser.add_argument("--kd", type=float, default=0.05,
                        help="PD velocity loop derivative gain")
    parser.add_argument("--motion-vel-threshold", type=float, default=0.02,
                        help="Min speed to accept a sweep measurement [rad/s]")

    # Sine excitation (for J)
    parser.add_argument("--skip-sine",       action="store_true",
                        help="Skip sine excitation (skip J identification)")
    parser.add_argument("--sine-duration-s", type=float, default=20.0)
    parser.add_argument("--sine-frequency",  type=float, default=0.45)
    parser.add_argument("--sine-amplitude",  type=float, default=1.6)

    # Regression
    parser.add_argument("--velocity-cutoff-hz",  type=float, default=8.0)
    parser.add_argument("--regression-min-speed", type=float, default=0.03)

    return parser.parse_args()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    args = parse_args()
    run_dir = Path(args.output_dir) / time.strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)

    interface = SerialMotorInterface(
        port=args.port, baudrate=args.baudrate, timeout=0.2, motor_id=args.motor_id
    )

    try:
        # ------------------------------------------------------------------
        # Phase 1 – Steady-state velocity sweep → Stribeck fit
        # ------------------------------------------------------------------
        print("=" * 60)
        print("Phase 1: Bidirectional steady-state velocity sweep")
        print("=" * 60)
        sweep = run_velocity_sweep(interface, args)

        if len(sweep["omega"]) < 4:
            raise RuntimeError("Too few valid sweep points for curve fitting. "
                               "Check hardware connection and PD gains.")

        print("\nFitting Stribeck model …")
        stribeck_params = fit_stribeck(sweep, args)

        print("\nStribeck parameters:")
        print(f"  τ_c  = {stribeck_params['tau_c']:.6f}  ± {stribeck_params['tau_c_std']:.6f}  [Nm]  (Coulomb)")
        print(f"  b    = {stribeck_params['b']:.6f}  ± {stribeck_params['b_std']:.6f}          (viscous)")
        print(f"  τ_s  = {stribeck_params['tau_s']:.6f}  ± {stribeck_params['tau_s_std']:.6f}  [Nm]  (static/stiction)")
        print(f"  v_s  = {stribeck_params['v_s']:.6f}  ± {stribeck_params['v_s_std']:.6f}  [rad/s] (Stribeck speed)")
        print(f"  R²   = {stribeck_params['r2']:.4f}")
        print(f"  RMSE = {stribeck_params['rmse']:.6f} Nm")

        write_csv(
            run_dir / "sweep_data.csv",
            ["omega_meas", "tau_meas"],
            [sweep["omega"], sweep["tau"]],
        )

        # Store Stribeck params for J estimation
        args._tau_c = stribeck_params["tau_c"]
        args._b     = stribeck_params["b"]
        args._tau_s = stribeck_params["tau_s"]
        args._v_s   = stribeck_params["v_s"]

        # ------------------------------------------------------------------
        # Phase 2 – Sine excitation → J
        # ------------------------------------------------------------------
        J = float("nan")
        sine_proc = None
        if not args.skip_sine:
            print("\n" + "=" * 60)
            print("Phase 2: Sinusoidal excitation for inertia J")
            print("=" * 60)
            raw_sine = run_sine_excitation(interface, args)
            print("Estimating inertia J …")
            sine_proc = estimate_inertia(raw_sine, args)
            J = sine_proc["J"]
            print(f"\nInertia:  J = {J:.6f}  (R²={sine_proc['r2']:.4f}, RMSE={sine_proc['rmse']:.6f})")

            write_csv(
                run_dir / "sine_data.csv",
                ["t", "tau_cmd", "tau_meas", "q", "dq"],
                [raw_sine["t"], raw_sine["tau_cmd"], raw_sine["tau_meas"],
                 raw_sine["q"], raw_sine["dq"]],
            )
            write_csv(
                run_dir / "sine_processed.csv",
                ["t", "tau_cmd", "tau_meas", "tau_pred", "dq_raw", "dq_filt", "ddq"],
                [sine_proc["t"], sine_proc["tau_cmd"], sine_proc["tau_meas"],
                 sine_proc["tau_pred"], sine_proc["dq_raw"], sine_proc["dq_filt"],
                 sine_proc["ddq"]],
            )

        # ------------------------------------------------------------------
        # Summary
        # ------------------------------------------------------------------
        print("\n" + "=" * 60)
        print("IDENTIFIED MOTOR MODEL")
        print("  τ = J·α + τ_c·sign(ω) + b·ω + (τ_s−τ_c)·exp(−(|ω|/v_s)²)·sign(ω)")
        print("=" * 60)
        print(f"  J    = {J:.6f}  [kg·m²]")
        print(f"  τ_c  = {stribeck_params['tau_c']:.6f}  [Nm]  (Coulomb)")
        print(f"  b    = {stribeck_params['b']:.6f}        (viscous  [Nm·s/rad])")
        print(f"  τ_s  = {stribeck_params['tau_s']:.6f}  [Nm]  (static / stiction)")
        print(f"  v_s  = {stribeck_params['v_s']:.6f}  [rad/s] (Stribeck transition speed)")
        print(f"\nData saved to: {run_dir}")

        # ------------------------------------------------------------------
        # Plots
        # ------------------------------------------------------------------
        plot_stribeck_curve(sweep, stribeck_params, run_dir)
        if sine_proc is not None:
            plot_sine_fit(sine_proc, run_dir)

    finally:
        interface.stop_motor()
        interface.close()


if __name__ == "__main__":
    main()