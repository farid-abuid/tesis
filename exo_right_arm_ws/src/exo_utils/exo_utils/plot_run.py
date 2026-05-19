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


def _plot_ee_xyz(
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


def _first_finite(arr: np.ndarray) -> float | None:
    for v in arr:
        if np.isfinite(v):
            return float(v)
    return None


def _plot_sliding_surface(
    t: np.ndarray,
    joints: list[str],
    rows: list[dict[str, str]],
    fieldnames: set[str],
    out_path: Path,
    title: str,
) -> bool:
    """All joints' sliding variable s on one axes vs time."""
    series: list[tuple[str, np.ndarray]] = []
    for j in joints:
        col = f'{j}_s'
        if col in fieldnames:
            arr = _as_array(rows, col)
            if _has_data(arr):
                series.append((j, arr))
    if not series:
        return False
    fig, ax = plt.subplots(figsize=(10, 5))
    for j, arr in series:
        ax.plot(t, arr, label=j)
    ax.axhline(0.0, color='k', lw=0.8, alpha=0.5)
    ax.set_xlabel('time [s]')
    ax.set_ylabel('s')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    return True


def _plot_phase(
    t: np.ndarray,
    joints: list[str],
    rows: list[dict[str, str]],
    fieldnames: set[str],
    out_path: Path,
    title: str,
) -> bool:
    """Per-joint sliding-surface phase plane.

    For s = e + lambda*int(e)  (integral SMC) the axes are (int(e), e).
    Otherwise s = de + lambda*e and the axes are (e, de). In both cases
    s = 0 is the straight line drawn over the trajectory, which is colored
    by time so convergence onto the surface is visible.
    """
    plottable: list[str] = []
    for j in joints:
        if f'{j}_s' in fieldnames or f'{j}_int_e' in fieldnames:
            plottable.append(j)
    if not plottable:
        return False

    n = len(plottable)
    fig, axes = plt.subplots(n, 1, figsize=(7, 5 * max(n, 1)))
    if n == 1:
        axes = [axes]
    sc = None
    for ax, j in zip(axes, plottable):
        q = _as_array(rows, f'{j}_q')
        q_des = _as_array(rows, f'{j}_q_des') if f'{j}_q_des' in fieldnames else None
        e = q - q_des if q_des is not None else q
        lam_arr = _as_array(rows, f'{j}_lambda') if f'{j}_lambda' in fieldnames else None
        lam = _first_finite(lam_arr) if lam_arr is not None else None

        if f'{j}_int_e' in fieldnames:
            x = _as_array(rows, f'{j}_int_e')
            y = e
            xlabel, ylabel = r'$\int e\,dt$ [rad·s]', 'e [rad]'
        else:
            dq = _as_array(rows, f'{j}_dq')
            dq_des = _as_array(rows, f'{j}_dq_des') if f'{j}_dq_des' in fieldnames else None
            x = e
            y = dq - dq_des if dq_des is not None else dq
            xlabel, ylabel = 'e [rad]', r'$\dot e$ [rad/s]'

        m = np.isfinite(x) & np.isfinite(y)
        if not m.any():
            ax.set_visible(False)
            continue
        sc = ax.scatter(x[m], y[m], c=t[m], s=6, cmap='viridis')
        ax.plot(x[m][0], y[m][0], 'o', color='red', label='start')
        if lam is not None and lam != 0.0:
            xr = np.array([np.nanmin(x[m]), np.nanmax(x[m])])
            ax.plot(xr, -lam * xr, 'r--', lw=1.5, label=f's = 0 (λ={lam:g})')
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.set_title(j)
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
    if sc is not None:
        fig.colorbar(sc, ax=axes, label='time [s]', shrink=0.6)
    fig.suptitle(title)
    fig.savefig(out_path, dpi=150, bbox_inches='tight')
    return True


# Barycentric parameter layout per link: 10 entries, grouped by physical type.
_THETA_GROUPS = [
    ('mass', 'Link mass m', [(0, 'm')]),
    ('com', 'Center of mass (m·c / m)',
     [(1, 'x_c'), (2, 'y_c'), (3, 'z_c')]),
    ('inertia', 'Second moments of mass (about link origin)',
     [(4, 'Jxx'), (5, 'Jyy'), (6, 'Jzz'), (7, 'Jxy'), (8, 'Jxz'), (9, 'Jyz')]),
]


def _plot_adaptive_params(
    t: np.ndarray,
    rows: list[dict[str, str]],
    fieldnames: set[str],
    prefix: str,
    out_dir: Path,
    title_prefix: str,
) -> list[Path]:
    """θ̂ evolution grouped by parameter type, 3 links overlaid, with θ̂₀ dashed."""
    if f'{prefix}theta_0' not in fieldnames:
        return []
    ntheta = 0
    while f'{prefix}theta_{ntheta}' in fieldnames:
        ntheta += 1
    n_links = ntheta // 10
    if n_links == 0:
        return []
    colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
    saved: list[Path] = []
    for fname, gtitle, comps in _THETA_GROUPS:
        nc = len(comps)
        fig, axes = plt.subplots(nc, 1, figsize=(10, 2.6 * nc), sharex=True)
        if nc == 1:
            axes = [axes]
        is_com = fname == 'com'  # plot m·c / m instead of the raw first moment
        for ax, (off, label) in zip(axes, comps):
            for link in range(n_links):
                idx = 10 * link + off
                col = f'{prefix}theta_{idx}'
                if col not in fieldnames:
                    continue
                c = colors[link % len(colors)]
                if is_com:
                    m_col = f'{prefix}theta_{10 * link}'
                    if m_col not in fieldnames:
                        continue
                    m = _as_array(rows, m_col)
                    series = np.divide(
                        _as_array(rows, col), m,
                        out=np.full_like(m, np.nan), where=m != 0.0)
                else:
                    series = _as_array(rows, col)
                ax.plot(t, series, color=c, label=f'link {link + 1}')
                c0 = f'{prefix}theta0_{idx}'
                if c0 in fieldnames:
                    if is_com:
                        m0_col = f'{prefix}theta0_{10 * link}'
                        m0 = _first_finite(_as_array(rows, m0_col)) \
                            if m0_col in fieldnames else None
                        n0 = _first_finite(_as_array(rows, c0))
                        v0 = (n0 / m0) if (m0 not in (None, 0.0)
                                           and n0 is not None) else None
                    else:
                        v0 = _first_finite(_as_array(rows, c0))
                    if v0 is not None:
                        ax.axhline(v0, color=c, ls='--', lw=1.0, alpha=0.7)
            ax.set_ylabel(label)
            ax.legend(loc='upper right', fontsize=8, ncol=max(n_links, 1))
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel('time [s]')
        fig.suptitle(f'{title_prefix}{gtitle}  (dashed = θ̂₀ seed)')
        fig.tight_layout()
        out_path = out_dir / f'theta_{fname}.png'
        fig.savefig(out_path, dpi=150)
        saved.append(out_path)
    return saved


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
            # ('_tau_ff', 'tau_ff'),
        ],
        tau_path, f'{prefix_txt}Joint torque', ylabel_unit='N·m',
    ):
        saved.append(tau_path)

    ee_path = out_dir / 'ee_position_xyz.png'
    if _plot_ee_xyz(t, rows, fieldnames, ee_prefix, ee_path, f'{prefix_txt}EE position vs desired'):
        saved.append(ee_path)

    s_path = out_dir / 'sliding_surface.png'
    if _plot_sliding_surface(t, joints, rows, fieldnames, s_path,
                             f'{prefix_txt}Sliding surface s vs time'):
        saved.append(s_path)

    phase_path = out_dir / 'sliding_phase_plane.png'
    if _plot_phase(t, joints, rows, fieldnames, phase_path,
                   f'{prefix_txt}Sliding-surface phase plane'):
        saved.append(phase_path)

    saved += _plot_adaptive_params(t, rows, fieldnames, ee_prefix, out_dir, prefix_txt)

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
