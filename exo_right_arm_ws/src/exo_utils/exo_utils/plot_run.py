"""Generate PNG plots from a telemetry CSV written by exo_data_logger.

Behaviour:
  - Single arm runs produce plots directly under ``plot_dir``. When the joints
    are prefixed (``left_`` / ``right_``) the plot titles include the arm tag.
  - Dual arm runs (joints prefixed with ``left_`` / ``right_``) produce a
    subdirectory per arm with its own figures.
  - Only columns actually present in the CSV are plotted (no fake q_des/q_cmd).
"""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path

import matplotlib

# Headless (no DISPLAY) falls back to Agg; interactive sessions keep the default.
if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')) \
        and not os.environ.get('MPLBACKEND'):
    matplotlib.use('Agg')

import matplotlib.pyplot as plt
import numpy as np


def _discover_joints(fieldnames: list[str]) -> list[str]:
    joints: list[str] = []
    seen: set[str] = set()
    for col in fieldnames:
        if col.endswith('_q') and not col.endswith('_dq'):
            j = col[:-2]
            if j and j not in seen:
                seen.add(j)
                joints.append(j)
    return joints


def _group_by_arm(joints: list[str]) -> dict[str, list[str]]:
    groups: dict[str, list[str]] = {}
    for j in joints:
        if j.startswith('left_'):
            groups.setdefault('left', []).append(j)
        elif j.startswith('right_'):
            groups.setdefault('right', []).append(j)
        else:
            groups.setdefault('main', []).append(j)
    return groups


def _as_array(rows: list[dict[str, str]], key: str) -> np.ndarray:
    out = np.empty(len(rows), dtype=float)
    for idx, r in enumerate(rows):
        v = r.get(key, '')
        if v in ('', 'nan', 'NaN', 'NAN'):
            out[idx] = np.nan
        else:
            try:
                out[idx] = float(v)
            except ValueError:
                out[idx] = np.nan
    return out


def _has_data(arr: np.ndarray) -> bool:
    return arr.size > 0 and np.isfinite(arr).any()


def _plot_stack(
    t: np.ndarray,
    joints: list[str],
    rows: list[dict[str, str]],
    fieldnames: set[str],
    suffix_map: list[tuple[str, str]],
    out_path: Path,
    title: str,
    ylabel_unit: str = '',
) -> bool:
    """Create a stacked per-joint figure. Returns True if the figure was saved.

    ``suffix_map`` is a list of (column_suffix, plot_label) pairs.
    Series with no finite samples are skipped; the whole figure is skipped if
    nothing would be drawn.
    """

    available: dict[str, dict[str, np.ndarray]] = {}
    for j in joints:
        series: dict[str, np.ndarray] = {}
        for suf, lab in suffix_map:
            col = f'{j}{suf}'
            if col in fieldnames:
                arr = _as_array(rows, col)
                if _has_data(arr):
                    series[lab] = arr
        if series:
            available[j] = series
    if not available:
        return False

    n = len(joints)
    fig, axes = plt.subplots(n, 1, figsize=(10, 2.5 * max(n, 1)), sharex=True)
    if n == 1:
        axes = [axes]
    for ax, j in zip(axes, joints):
        series = available.get(j, {})
        for lab, arr in series.items():
            ax.plot(t, arr, label=lab)
        ax.set_ylabel(f'{j}{" [" + ylabel_unit + "]" if ylabel_unit else ""}')
        if series:
            ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel('time [s]')
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    return True


def _plot_task_space(
    t: np.ndarray,
    rows: list[dict[str, str]],
    fieldnames: set[str],
    prefix: str,
    out_path: Path,
    title: str,
) -> bool:
    need = {f'{prefix}ee_x', f'{prefix}ee_y', f'{prefix}ee_z'}
    if not need.issubset(fieldnames):
        return False
    axes_names = ['x', 'y', 'z']
    fig, axs = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    for ax, axis in zip(axs, axes_names):
        a = _as_array(rows, f'{prefix}ee_{axis}')
        d = _as_array(rows, f'{prefix}ee_des_{axis}')
        if _has_data(a):
            ax.plot(t, a, label=f'{axis} actual')
        if _has_data(d):
            ax.plot(t, d, label=f'{axis} des')
        ax.set_ylabel(axis)
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
    axs[-1].set_xlabel('time [s]')
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    return True


def _generate_for_group(
    arm: str,
    joints: list[str],
    t: np.ndarray,
    rows: list[dict[str, str]],
    fieldnames: set[str],
    out_dir: Path,
    arm_label: str | None,
) -> list[Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    prefix_txt = f'[{arm_label}] ' if arm_label else ''
    ee_prefix = f'{arm}_' if arm != 'main' else ''
    saved: list[Path] = []

    pos_path = out_dir / 'joint_position.png'
    if _plot_stack(
        t, joints, rows, fieldnames,
        [('_q', 'q'), ('_q_cmd', 'q_cmd'), ('_q_des', 'q_des')],
        pos_path, f'{prefix_txt}Joint position', ylabel_unit='rad',
    ):
        saved.append(pos_path)

    vel_path = out_dir / 'joint_velocity.png'
    if _plot_stack(
        t, joints, rows, fieldnames,
        [('_dq', 'dq'), ('_dq_cmd', 'dq_cmd'), ('_dq_des', 'dq_des')],
        vel_path, f'{prefix_txt}Joint velocity', ylabel_unit='rad/s',
    ):
        saved.append(vel_path)

    tau_path = out_dir / 'joint_torque.png'
    if _plot_stack(
        t, joints, rows, fieldnames,
        [
            ('_tau_meas', 'tau_meas'),
            ('_tau_cmd', 'tau_cmd'),
            ('_tau_cmd_in', 'tau_cmd_in'),
            ('_tau_ff', 'tau_ff'),
        ],
        tau_path, f'{prefix_txt}Joint torque', ylabel_unit='N·m',
    ):
        saved.append(tau_path)

    ee_path = out_dir / 'task_space_xyz.png'
    if _plot_task_space(t, rows, fieldnames, ee_prefix, ee_path, f'{prefix_txt}Task-space position vs desired'):
        saved.append(ee_path)

    return saved


def generate_plots(csv_path: str, plot_dir: str, show: bool = False) -> list[Path]:
    csv_p = Path(csv_path)
    out_p = Path(plot_dir)
    out_p.mkdir(parents=True, exist_ok=True)

    with csv_p.open(newline='') as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        if not rows or reader.fieldnames is None:
            return []
        fieldnames = set(reader.fieldnames)

    t = _as_array(rows, 'time_sec')
    joints = _discover_joints(list(fieldnames))
    if not joints:
        return []

    groups = _group_by_arm(joints)
    multi_arm = len(groups) > 1
    saved: list[Path] = []
    for arm, arm_joints in groups.items():
        arm_joints.sort()
        arm_out = out_p / arm if multi_arm else out_p
        # Always tag plots with the arm when it has a name (left/right), even on
        # single-arm runs. Legacy 'main' (unprefixed) runs stay unlabeled.
        label = arm if (multi_arm or arm != 'main') else None
        saved += _generate_for_group(arm, arm_joints, t, rows, fieldnames, arm_out, label)

    if show and saved:
        plt.show()
    else:
        plt.close('all')
    return saved


def main() -> None:
    p = argparse.ArgumentParser(description='Plot telemetry CSV from exo_data_logger')
    p.add_argument('csv', type=str, help='Path to data.csv')
    p.add_argument(
        '-o',
        '--output-dir',
        type=str,
        default='',
        help='Plot output directory (default: sibling plots/ next to csv)',
    )
    p.add_argument('--no-show', action='store_true', help='Only save; do not open windows')
    args = p.parse_args()
    csv_path = Path(args.csv)
    out = Path(args.output_dir) if args.output_dir else csv_path.parent / 'plots'
    saved = generate_plots(str(csv_path), str(out), show=not args.no_show)
    print(f'Plots written to {out}')
    for s in saved:
        print(f'  - {s}')


if __name__ == '__main__':
    main()
