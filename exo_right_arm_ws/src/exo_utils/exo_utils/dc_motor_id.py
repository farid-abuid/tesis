#!/usr/bin/env python3
"""Bench DC motor parameter identification over Teensy serial."""

import argparse
import csv
import math
import struct
import time
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import serial

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
    """Implements the Teensy frame format used in teensy_hardware.ino."""

    def __init__(
        self, port: str, baudrate: int, timeout: float, motor_id: int, retries: int = 3
    ):
        self.motor_id = motor_id
        self.serial = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        self.retries = max(1, retries)

    @staticmethod
    def _checksum(payload: bytes) -> int:
        cs = 0
        for b in payload:
            cs ^= b
        return cs

    def _build_command(self, cmd_type: int, command_value: float) -> bytes:
        meta = ((cmd_type & 0x0F) << 4) | 0x01
        payload_wo_cs = struct.pack("<BBf", meta, self.motor_id, float(command_value))
        checksum = self._checksum(payload_wo_cs)
        return bytes([HEADER1, HEADER2]) + payload_wo_cs + bytes([checksum])

    def _read_exact(self, n_bytes: int) -> bytes:
        data = self.serial.read(n_bytes)
        if len(data) != n_bytes:
            raise TimeoutError(f"Expected {n_bytes} bytes, received {len(data)}")
        return data

    def _read_status_frame(self) -> MotorStatus:
        while True:
            b = self._read_exact(1)[0]
            if b != HEADER1:
                continue
            b2 = self._read_exact(1)[0]
            if b2 != HEADER2:
                continue
            raw = self._read_exact(STATUS_STRUCT.size)
            motor_id, tau_meas, dq, q = STATUS_STRUCT.unpack(raw)
            return MotorStatus(motor_id=motor_id, tau_meas=tau_meas, dq=dq, q=q)

    def command_torque_and_read(self, tau_cmd: float) -> MotorStatus:
        frame = self._build_command(CMD_TORQUE, tau_cmd)
        last_error = None
        for _ in range(self.retries):
            self.serial.write(frame)
            try:
                status = self._read_status_frame()
                if status.motor_id != self.motor_id:
                    raise TimeoutError(
                        f"Received motor_id={status.motor_id}, expected {self.motor_id}"
                    )
                return status
            except TimeoutError as exc:
                last_error = exc
                self.serial.reset_input_buffer()
        raise TimeoutError(f"Serial exchange failed after {self.retries} retries: {last_error}")

    def stop_motor(self, stop_hold_s: float = 0.2, repeats: int = 20) -> None:
        _ = repeats
        try:
            self.command_torque_and_read(0.0)
        except TimeoutError:
            pass
        time.sleep(stop_hold_s)
           

    def close(self) -> None:
        if self.serial and self.serial.is_open:
            self.serial.close()


def low_pass_filter(signal: np.ndarray, dt: float, cutoff_hz: float) -> np.ndarray:
    if cutoff_hz <= 0.0:
        return signal.copy()
    rc = 1.0 / (2.0 * math.pi * cutoff_hz)
    alpha = dt / (rc + dt)
    out = np.empty_like(signal)
    out[0] = signal[0]
    for i in range(1, signal.size):
        out[i] = out[i - 1] + alpha * (signal[i] - out[i - 1])
    return out


def run_stiction_test(interface: SerialMotorInterface, args: argparse.Namespace) -> dict:
    thresholds = []
    for trial_idx in range(args.stiction_trials):
        current = args.stiction_start_torque
        start = time.monotonic()
        consecutive_motion = 0
        q0 = None
        detected = False

        while time.monotonic() - start < args.stiction_max_time_s:
            status = interface.command_torque_and_read(current)
            if q0 is None:
                q0 = status.q

            moved_speed = abs(status.dq) >= args.motion_vel_threshold
            moved_angle = abs(status.q - q0) >= args.motion_angle_threshold
            if moved_speed:
                consecutive_motion += 1
            else:
                consecutive_motion = 0

            if consecutive_motion >= args.motion_consecutive_samples:
                thresholds.append(current)
                detected = True
                break

            current = min(current + args.stiction_ramp_rate * args.dt, args.max_current)
            time.sleep(args.dt)

            #print(f"Trial {trial_idx + 1}/{args.stiction_trials} current: {current:.4f} Nm")
        interface.stop_motor(stop_hold_s=args.trial_settle_s)
        if not detected:
            thresholds.append(args.max_current)
        print(f"Trial {trial_idx + 1}/{args.stiction_trials} threshold: {thresholds[-1]:.4f} Nm")

    stiction_mean = float(np.mean(thresholds))
    stiction_std = float(np.std(thresholds, ddof=1)) if len(thresholds) > 1 else 0.0
    return {
        "thresholds": thresholds,
        "mean": stiction_mean,
        "std": stiction_std,
    }


def run_sine_excitation(interface: SerialMotorInterface, args: argparse.Namespace) -> dict:
    amp = args.sine_amplitude
    omega = 2.0 * math.pi * args.sine_frequency
    n_samples = int(args.sine_duration_s / args.dt)
    t0 = time.monotonic()

    t_arr = np.zeros(n_samples, dtype=float)
    tau_cmd_arr = np.zeros(n_samples, dtype=float)
    tau_meas_arr = np.zeros(n_samples, dtype=float)
    q_arr = np.zeros(n_samples, dtype=float)
    dq_arr = np.zeros(n_samples, dtype=float)

    for k in range(n_samples):
        tk = time.monotonic() - t0
        tau_cmd = args.sine_amplitude * math.sin(omega * tk)
        status = interface.command_torque_and_read(tau_cmd)

        t_arr[k] = tk
        tau_cmd_arr[k] = tau_cmd
        tau_meas_arr[k] = status.tau_meas
        q_arr[k] = status.q
        dq_arr[k] = status.dq
        time.sleep(args.dt)

    interface.stop_motor(stop_hold_s=0.3)
    n_discard = 300
    return {
        "t": t_arr[n_discard:-n_discard],
        "tau_cmd": tau_cmd_arr[n_discard:-n_discard],
        "tau_meas": tau_meas_arr[n_discard:-n_discard],
        "q": q_arr[n_discard:-n_discard],
        "dq": dq_arr[n_discard:-n_discard],
        "used_amplitude": amp,
    }


def estimate_params_regression(
    data: dict, args: argparse.Namespace, stiction_mean: float
) -> dict:
    t = data["t"]
    dq = data["dq"]
    tau_meas = data["tau_meas"]
    tau_cmd = data["tau_cmd"]
    dt_mean = float(np.mean(np.diff(t)))

    dq_f = low_pass_filter(dq, dt_mean, args.velocity_cutoff_hz)
    ddq = np.gradient(dq_f, t)
    vel_sign = np.sign(dq_f)

    mask = np.abs(dq_f) > args.regression_min_speed

    X = np.column_stack([ddq[mask], dq_f[mask], vel_sign[mask]])
    y = tau_meas[mask]

    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    J, b, tau_c = [float(v) for v in beta]

    tau_pred = J * ddq + b * dq_f + tau_c * np.sign(dq_f)
    residual = tau_meas - tau_pred
    ss_res = float(np.sum(residual**2))
    ss_tot = float(np.sum((tau_meas - np.mean(tau_meas)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else float("nan")
    rmse = float(np.sqrt(np.mean(residual**2)))

    processed = {
        "t": t,
        "tau_cmd": tau_cmd,
        "tau_meas": tau_meas,
        "tau_pred": tau_pred,
        "q": data["q"],
        "dq_raw": dq,
        "dq_filt": dq_f,
        "ddq": ddq,
    }
    params = {
        "J": J,
        "b": b,
        "tau_c": tau_c,
        "stiction_mean": stiction_mean,
        "r2": r2,
        "rmse": rmse,
        "used_amplitude": data["used_amplitude"],
    }
    return {"processed": processed, "params": params}


def write_csv(path: Path, header: list[str], columns: list[np.ndarray]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for row in zip(*columns):
            writer.writerow([f"{float(v):.9f}" for v in row])


def plot_results(processed: dict, params: dict) -> None:
    t = processed["t"]
    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    ax[0].plot(t, processed["tau_cmd"], label="tau_cmd", linewidth=1.0)
    ax[0].plot(t, processed["tau_meas"], label="tau_meas", linewidth=1.0)
    ax[0].set_ylabel("Torque [Nm]")
    ax[0].legend()
    ax[0].grid(True)

    ax[1].plot(t, processed["dq_raw"], label="dq_raw", linewidth=1.0, alpha=0.7)
    ax[1].plot(t, processed["dq_filt"], label="dq_filt", linewidth=1.0)
    ax[1].set_ylabel("Velocity [rad/s]")
    ax[1].legend()
    ax[1].grid(True)

    ax[2].plot(t, processed["ddq"], label="ddq", linewidth=1.0)
    ax[2].set_ylabel("Accel [rad/s^2]")
    ax[2].set_xlabel("Time [s]")
    ax[2].legend()
    ax[2].grid(True)

    fig2, ax2 = plt.subplots(1, 1, figsize=(6, 6))
    ax2.scatter(processed["tau_meas"], processed["tau_pred"], s=8, alpha=0.6)
    minv = float(min(np.min(processed["tau_meas"]), np.min(processed["tau_pred"])))
    maxv = float(max(np.max(processed["tau_meas"]), np.max(processed["tau_pred"])))
    ax2.plot([minv, maxv], [minv, maxv], "r--", linewidth=1.0)
    ax2.set_xlabel("Measured torque [Nm]")
    ax2.set_ylabel("Predicted torque [Nm]")
    ax2.set_title(f"Measured vs Predicted (R2={params['r2']:.3f})")
    ax2.grid(True)

    plt.tight_layout()
    plt.show()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="DC motor parameter identification")
    parser.add_argument("--port", type=str, default="/dev/teensy_motor")
    parser.add_argument("--baudrate", type=int, default=460800)
    parser.add_argument("--motor-id", type=int, required=True)
    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument("--max-current", type=float, default=5.0)
    parser.add_argument("--output-dir", type=str, default="motor_id_runs")

    parser.add_argument("--stiction-trials", type=int, default=0)
    parser.add_argument("--stiction-start-torque", type=float, default=0.6)
    parser.add_argument("--stiction-ramp-rate", type=float, default=0.1)
    parser.add_argument("--stiction-max-time-s", type=float, default=20.0)
    parser.add_argument("--trial-settle-s", type=float, default=5.0)
    parser.add_argument("--motion-vel-threshold", type=float, default=0.15)
    parser.add_argument("--motion-angle-threshold", type=float, default=0.005)
    parser.add_argument("--motion-consecutive-samples", type=int, default=5)

    parser.add_argument("--sine-duration-s", type=float, default=20.0)
    parser.add_argument("--sine-frequency", type=float, default=0.45)
    parser.add_argument("--sine-amplitude", type=float, default=1.6)

    parser.add_argument("--velocity-cutoff-hz", type=float, default=8.0)
    parser.add_argument("--regression-min-speed", type=float, default=0.03)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    run_dir = Path(args.output_dir) / time.strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)

    interface = SerialMotorInterface(
        port=args.port, baudrate=args.baudrate, timeout=0.2, motor_id=args.motor_id
    )
    try:
        print("Running stiction test...")
        stiction = run_stiction_test(interface, args)
        print(
            f"Stiction mean={stiction['mean']:.4f} Nm, std={stiction['std']:.4f} Nm"
        )

        print("Running sinusoidal excitation...")
        raw = run_sine_excitation(interface, args)

        print("Estimating J, b, tau_c...")
        fit = estimate_params_regression(raw, args, stiction["mean"])
        params = fit["params"]
        processed = fit["processed"]

        write_csv(
            run_dir / "raw_data.csv",
            ["t", "tau_cmd", "tau_meas", "q", "dq"],
            [raw["t"], raw["tau_cmd"], raw["tau_meas"], raw["q"], raw["dq"]],
        )
        write_csv(
            run_dir / "processed_data.csv",
            ["t", "tau_cmd", "tau_meas", "tau_pred", "q", "dq_raw", "dq_filt", "ddq"],
            [
                processed["t"],
                processed["tau_cmd"],
                processed["tau_meas"],
                processed["tau_pred"],
                processed["q"],
                processed["dq_raw"],
                processed["dq_filt"],
                processed["ddq"],
            ],
        )

        print("Identified parameters:")
        print(f"  J        = {params['J']:.6f}")
        print(f"  b        = {params['b']:.6f}")
        print(f"  tau_c    = {params['tau_c']:.6f}")
        print(f"  stiction = {params['stiction_mean']:.6f} +/- {stiction['std']:.6f}")
        print(f"  R2       = {params['r2']:.4f}")
        print(f"  RMSE     = {params['rmse']:.6f}")
        print(f"Data saved to: {run_dir}")

        plot_results(processed, params)
    finally:
        interface.stop_motor()
        interface.close()


if __name__ == "__main__":
    main()
