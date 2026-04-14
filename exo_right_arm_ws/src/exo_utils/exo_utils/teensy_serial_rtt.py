#!/usr/bin/env python3
"""Measure Teensy serial round-trip time (host clock).

Sends one command frame for N motors, blocks until one status frame with N
MotorStatus2 structs is received, repeats. Latency is measured on the PC
between write and full read (no Teensy t_cmd_sent_us pairing).
"""

from __future__ import annotations

import argparse
import statistics
import struct
import sys
import time

import serial

HEADER1 = 0xAA
HEADER2 = 0x55
STATUS_STRUCT = struct.Struct("<Bfff")
N_MOTORS_MAX = 6


def checksum_xor(payload: bytes) -> int:
    cs = 0
    for b in payload:
        cs ^= b
    return cs


def build_command(cmd_type: int, motor_ids: list[int], values: list[float]) -> bytes:
    n = len(motor_ids)
    if len(values) != n:
        raise ValueError("motor_ids and values must have the same length")
    if not (1 <= n <= N_MOTORS_MAX):
        raise ValueError(f"n_motors must be 1..{N_MOTORS_MAX}, got {n}")
    meta = ((cmd_type & 0x0F) << 4) | (n & 0x0F)
    body = struct.pack("<B", meta)
    for mid, val in zip(motor_ids, values):
        body += struct.pack("<B", mid & 0xFF) + struct.pack("<f", float(val))
    cs = checksum_xor(body)
    return bytes([HEADER1, HEADER2]) + body + bytes([cs])


def read_status_frame(ser: serial.Serial, n_motors: int) -> bytes:
    """Read AA 55 + n_motors * MotorStatus2."""
    payload_len = STATUS_STRUCT.size * n_motors
    while True:
        b = ser.read(1)
        if not b:
            raise TimeoutError("Timeout waiting for byte (sync HEADER1)")
        if b[0] != HEADER1:
            continue
        b2 = ser.read(1)
        if len(b2) != 1:
            raise TimeoutError("Timeout waiting for HEADER2")
        if b2[0] != HEADER2:
            continue
        rest = ser.read(payload_len)
        if len(rest) != payload_len:
            raise TimeoutError(
                f"Timeout reading status ({len(rest)}/{payload_len} bytes)"
            )
        return rest


def _parse_csv_ints(s: str) -> list[int]:
    parts = [p.strip() for p in s.split(",") if p.strip()]
    return [int(p, 10) for p in parts]


def _parse_csv_floats(s: str) -> list[float]:
    parts = [p.strip() for p in s.split(",") if p.strip()]
    return [float(p) for p in parts]


def _resolve_motors(args: argparse.Namespace) -> tuple[list[int], list[float]]:
    if args.n_motors == 1:
        return [args.motor_id], [args.value]
    ids_s = args.motor_ids
    vals_s = args.values
    if ids_s is None:
        motor_ids = list(range(1, args.n_motors + 1))
    else:
        motor_ids = _parse_csv_ints(ids_s)
    if vals_s is None:
        values = [0.0] * args.n_motors
    else:
        values = _parse_csv_floats(vals_s)
    if len(motor_ids) != args.n_motors:
        raise ValueError(
            f"--motor-ids must list {args.n_motors} integers (got {len(motor_ids)})"
        )
    if len(values) != args.n_motors:
        raise ValueError(
            f"--values must list {args.n_motors} floats (got {len(values)})"
        )
    return motor_ids, values


def run_bench(args: argparse.Namespace) -> int:
    try:
        motor_ids, values = _resolve_motors(args)
    except ValueError as e:
        print(e, file=sys.stderr)
        return 1

    n_motors = len(motor_ids)

    try:
        ser = serial.Serial(port=args.port, baudrate=args.baud, timeout=args.timeout)
    except serial.SerialException as e:
        print(f"Could not open {args.port}: {e}", file=sys.stderr)
        return 1

    ser.reset_input_buffer()
    ser.reset_output_buffer()

    cmd = build_command(args.cmd_type, motor_ids, values)

    for _ in range(args.warmup):
        ser.write(cmd)
        ser.flush()
        read_status_frame(ser, n_motors)

    samples_us: list[float] = []
    for _ in range(args.samples):
        ser.reset_input_buffer()

        t0 = time.perf_counter()
        ser.write(cmd)
        ser.flush()
        read_status_frame(ser, n_motors)
        t1 = time.perf_counter()

        samples_us.append((t1 - t0) * 1e6)
        if args.pause_ms > 0:
            time.sleep(args.pause_ms / 1000.0)

    ser.close()

    avg = statistics.mean(samples_us)
    med = statistics.median(samples_us)
    stdev = statistics.stdev(samples_us) if len(samples_us) > 1 else 0.0
    print(
        f"samples={args.samples} n_motors={n_motors} cmd_type={args.cmd_type} "
        f"motor_ids={motor_ids} values={values}"
    )
    print(
        f"RTT us: mean={avg:.1f} median={med:.1f} stdev={stdev:.1f} "
        f"min={min(samples_us):.1f} max={max(samples_us):.1f}"
    )
    return 0


def main() -> int:
    p = argparse.ArgumentParser(
        description="Teensy serial round-trip benchmark (1 or more motors per frame)."
    )
    p.add_argument("--port", default="/dev/teensy_motor", help="Serial device")
    p.add_argument("--baud", type=int, default=460800)
    p.add_argument("--timeout", type=float, default=0.5, help="Per-byte read timeout (s)")
    p.add_argument(
        "--n-motors",
        type=int,
        default=1,
        metavar="N",
        help=f"Motors per command/response (1..{N_MOTORS_MAX}; must match Teensy frame)",
    )
    p.add_argument(
        "--motor-id",
        type=int,
        default=1,
        help="CAN motor ID (only when --n-motors is 1)",
    )
    p.add_argument(
        "--cmd-type",
        type=int,
        default=1,
        choices=(1, 2, 3),
        help="1=torque, 2=speed, 3=position (Teensy protocol)",
    )
    p.add_argument(
        "--value",
        type=float,
        default=0.0,
        help="Command float when --n-motors is 1 (Nm, rad/s, or rad)",
    )
    p.add_argument(
        "--motor-ids",
        type=str,
        default=None,
        help='Comma-separated IDs when --n-motors >1 (default: 1,2,...,N)',
    )
    p.add_argument(
        "--values",
        type=str,
        default=None,
        help='Comma-separated floats when --n-motors >1 (default: N zeros)',
    )
    p.add_argument("--samples", type=int, default=200)
    p.add_argument("--warmup", type=int, default=5)
    p.add_argument(
        "--pause-ms",
        type=float,
        default=2.0,
        help="Sleep between samples (ms); use >0 to reduce host/USB queue effects",
    )
    args = p.parse_args()
    if not (1 <= args.n_motors <= N_MOTORS_MAX):
        print(f"--n-motors must be 1..{N_MOTORS_MAX}", file=sys.stderr)
        return 1
    return run_bench(args)


if __name__ == "__main__":
    raise SystemExit(main())
