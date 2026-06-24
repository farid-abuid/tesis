"""Integral tracking-error metrics (IAE, ISE, ITAE, ITSE) from a telemetry CSV.

These are whole-run performance summaries, so they are computed offline from the
CSV written by ``exo_data_logger`` rather than in the real-time control loop.

For each joint the tracking error is ``e = q_des - q`` and, integrating against
the *actual* sample timestamps (trapezoidal rule, robust to loop jitter):

  IAE  = ∫ |e|     dt        ISE  = ∫ e²       dt
  ITAE = ∫ t·|e|   dt        ITSE = ∫ t·e²     dt

The time weight ``t`` in ITAE/ITSE is measured from the start of the evaluation
window (``t = 0`` there), so windowing does not silently reward shorter runs.
Compare controllers over the same trajectory and the same window.

Only joints that have both a ``_q`` and a ``_q_des`` column contribute (the
tracking error is undefined otherwise).
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np

# Reuse the CSV/joint discovery helpers so both tools agree on column parsing.
from exo_utils.plot_run import (
    _arm_suffix,
    _as_array,
    _disp_joint,
    _discover_joints,
    _group_by_arm,
)

_METRIC_NAMES = ('IAE', 'ISE', 'ITAE', 'ITSE')


def _trapz(y: np.ndarray, t: np.ndarray) -> float:
    """∫ y dt by the trapezoidal rule over (possibly uneven) timestamps."""
    if y.size < 2:
        return 0.0
    return float(np.sum(0.5 * (y[:-1] + y[1:]) * np.diff(t)))


def integral_metrics(
    t: np.ndarray,
    q: np.ndarray,
    q_des: np.ndarray,
    start: float | None = None,
    end: float | None = None,
    trim: float = 0.0,
) -> dict[str, float] | None:
    """IAE/ISE/ITAE/ITSE for one joint, or ``None`` if there is no usable data.

    ``start``/``end`` are absolute ``time_sec`` bounds; ``trim`` drops that many
    seconds after the (clamped) window start. The time weight is reset so that
    ``t = 0`` at the start of the resulting window.
    """
    mask = np.isfinite(t) & np.isfinite(q) & np.isfinite(q_des)
    if start is not None:
        mask &= t >= start
    if end is not None:
        mask &= t <= end
    tw, qw, qdw = t[mask], q[mask], q_des[mask]
    if tw.size and trim > 0.0:
        keep = tw >= tw[0] + trim
        tw, qw, qdw = tw[keep], qw[keep], qdw[keep]
    if tw.size < 2:
        return None

    e = qdw - qw
    t_rel = tw - tw[0]
    return {
        'n': tw.size,
        't_start': float(tw[0]),
        't_end': float(tw[-1]),
        'duration': float(tw[-1] - tw[0]),
        'IAE': _trapz(np.abs(e), tw),
        'ISE': _trapz(e ** 2, tw),
        'ITAE': _trapz(t_rel * np.abs(e), tw),
        'ITSE': _trapz(t_rel * e ** 2, tw),
    }


def compute_metrics(
    csv_path: str,
    start: float | None = None,
    end: float | None = None,
    trim: float = 0.0,
) -> list[dict]:
    """Per-joint metric rows for *csv_path*, plus an aggregate row per arm.

    Each row is a dict with keys: ``arm``, ``joint``, ``n``, ``t_start``,
    ``t_end``, ``duration`` and the four metric names. The aggregate row
    (``joint='__all__'``) sums each integral metric over the arm's joints.
    """
    csv_p = Path(csv_path)
    with csv_p.open(newline='') as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        if not rows or reader.fieldnames is None:
            return []
        fieldnames = set(reader.fieldnames)

    t = _as_array(rows, 'time_sec')
    joints = _discover_joints(list(fieldnames))
    groups = _group_by_arm(joints)

    out: list[dict] = []
    for arm, arm_joints in groups.items():
        arm_joints.sort()
        joint_results: list[dict] = []
        for j in arm_joints:
            if f'{j}_q_des' not in fieldnames:
                continue  # no reference -> tracking error undefined
            res = integral_metrics(
                t, _as_array(rows, f'{j}_q'), _as_array(rows, f'{j}_q_des'),
                start=start, end=end, trim=trim,
            )
            if res is None:
                continue
            res.update(arm=arm, joint=j)
            joint_results.append(res)
        if not joint_results:
            continue
        out.extend(joint_results)
        agg = {m: sum(r[m] for r in joint_results) for m in _METRIC_NAMES}
        agg.update(
            arm=arm, joint='__all__',
            n=max(r['n'] for r in joint_results),
            t_start=min(r['t_start'] for r in joint_results),
            t_end=max(r['t_end'] for r in joint_results),
            duration=max(r['duration'] for r in joint_results),
        )
        out.append(agg)
    return out


_FIELDS = ('arm', 'joint', 'n', 't_start', 't_end', 'duration', *_METRIC_NAMES)


def write_metrics_csv(results: list[dict], out_path: str | Path) -> Path:
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=_FIELDS)
        w.writeheader()
        for r in results:
            w.writerow({k: r.get(k, '') for k in _FIELDS})
    return out_path


def format_table(results: list[dict]) -> str:
    """Human-readable table grouped by arm, with the aggregate row last."""
    if not results:
        return '(no joints with a reference trajectory; nothing to evaluate)'
    lines: list[str] = []
    by_arm: dict[str, list[dict]] = {}
    for r in results:
        by_arm.setdefault(r['arm'], []).append(r)
    for arm, rs in by_arm.items():
        suffix = _arm_suffix(arm if arm != 'main' else None)
        lines.append(f'Métricas de seguimiento{suffix}'.rstrip())
        hdr = f'  {"articulación":<14}{"IAE":>12}{"ISE":>12}{"ITAE":>12}{"ITSE":>12}'
        lines.append(hdr)
        lines.append('  ' + '-' * (len(hdr) - 2))
        for r in rs:
            name = 'TOTAL' if r['joint'] == '__all__' else _disp_joint(r['joint'])
            lines.append(
                f'  {name:<14}{r["IAE"]:>12.5g}{r["ISE"]:>12.5g}'
                f'{r["ITAE"]:>12.5g}{r["ITSE"]:>12.5g}'
            )
        lines.append('')
    return '\n'.join(lines).rstrip()


def main() -> None:
    p = argparse.ArgumentParser(
        description='Compute IAE/ISE/ITAE/ITSE tracking-error metrics from a '
                    'telemetry CSV written by exo_data_logger',
    )
    p.add_argument('csv', type=str, help='Path to data.csv')
    p.add_argument('-o', '--output', type=str, default='',
                   help='Output CSV path (default: metrics.csv next to the input)')
    p.add_argument('--start', type=float, default=None,
                   help='Window start in time_sec (default: first sample)')
    p.add_argument('--end', type=float, default=None,
                   help='Window end in time_sec (default: last sample)')
    p.add_argument('--trim', type=float, default=0.0,
                   help='Drop this many seconds after the window start '
                        '(e.g. to skip the startup transient)')
    p.add_argument('--no-write', action='store_true',
                   help='Print the table only; do not write a CSV')
    args = p.parse_args()

    results = compute_metrics(args.csv, start=args.start, end=args.end, trim=args.trim)
    print(format_table(results))
    if results and not args.no_write:
        out = Path(args.output) if args.output else Path(args.csv).parent / 'metrics.csv'
        write_metrics_csv(results, out)
        print(f'\nMetrics written to {out}')


if __name__ == '__main__':
    main()
