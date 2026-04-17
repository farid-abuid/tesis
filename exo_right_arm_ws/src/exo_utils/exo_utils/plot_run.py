"""Generate PNG plots from a telemetry CSV written by exo_data_logger."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib

#matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def _discover_joints(fieldnames: list[str]) -> list[str]:
    joints: list[str] = []
    seen: set[str] = set()
    suf = '_q_des'
    for col in fieldnames:
        if col.endswith(suf):
            j = col[: -len(suf)]
            if j not in seen:
                seen.add(j)
                joints.append(j)
    return joints


def _discover_command_only_joints(fieldnames: list[str]) -> list[str]:
    joints: list[str] = []
    seen: set[str] = set()
    suf = '_q_cmd'
    for col in fieldnames:
        if col.endswith(suf):
            j = col[: -len(suf)]
            if j not in seen:
                seen.add(j)
                joints.append(j)
    return joints


def _has_field(rows: list[dict[str, str]], key: str) -> bool:
    return any(r.get(key, '') not in ('', 'nan', 'NaN') for r in rows)


def generate_plots(csv_path: str, plot_dir: str) -> None:
    csv_p = Path(csv_path)
    out_p = Path(plot_dir)
    out_p.mkdir(parents=True, exist_ok=True)

    with csv_p.open(newline='') as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        if not rows or reader.fieldnames is None:
            return
        fieldnames = list(reader.fieldnames)

    t = np.array([float(r['time_sec']) for r in rows], dtype=float)
    joints = _discover_joints(fieldnames)
    command_only_joints = _discover_command_only_joints(fieldnames) if not joints else []
    if not joints:
        joints = command_only_joints
    if not joints:
        return

    n = len(joints)
    has_q = all(f'{j}_q' in fieldnames and f'{j}_q_des' in fieldnames for j in joints)
    has_dq = all(f'{j}_dq' in fieldnames and f'{j}_dq_des' in fieldnames for j in joints)
    has_tau = all(f'{j}_tau_cmd' in fieldnames for j in joints)

    if has_q:
        fig, axes = plt.subplots(n, 1, figsize=(10, 2.5 * n), sharex=True)
        if n == 1:
            axes = [axes]
        for ax, j in zip(axes, joints):
            q = np.array([float(r[f'{j}_q']) for r in rows])
            qd = np.array([float(r[f'{j}_q_des']) for r in rows])
            ax.plot(t, q, label='q')
            ax.plot(t, qd, label='q_des')
            ax.set_ylabel(j)
            ax.legend(loc='upper right')
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel('time [s]')
        fig.suptitle('Joint position vs desired')
        fig.tight_layout()
        fig.savefig(out_p / 'joint_pos_vs_desired.png', dpi=150)
        plt.show()
        plt.close(fig)

    if has_dq:
        fig, axes = plt.subplots(n, 1, figsize=(10, 2.5 * n), sharex=True)
        if n == 1:
            axes = [axes]
        for ax, j in zip(axes, joints):
            dq = np.array([float(r[f'{j}_dq']) for r in rows])
            dqd = np.array([float(r[f'{j}_dq_des']) for r in rows])
            ax.plot(t, dq, label='dq')
            ax.plot(t, dqd, label='dq_des')
            ax.set_ylabel(j)
            ax.legend(loc='upper right')
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel('time [s]')
        fig.suptitle('Joint velocity vs desired')
        fig.tight_layout()
        fig.savefig(out_p / 'joint_vel_vs_desired.png', dpi=150)
        plt.show()
        plt.close(fig)

    if has_tau:
        fig, axes = plt.subplots(n, 1, figsize=(10, 2.5 * n), sharex=True)
        if n == 1:
            axes = [axes]
        for ax, j in zip(axes, joints):
            tau = np.array([float(r[f'{j}_tau_cmd']) for r in rows])
            ax.plot(t, tau, label='tau_cmd')
            ax.set_ylabel(j)
            ax.legend(loc='upper right')
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel('time [s]')
        fig.suptitle('Joint commanded torque')
        fig.tight_layout()
        fig.savefig(out_p / 'joint_tau_vs_cmd.png', dpi=150)
        plt.show()
        plt.close(fig)

    if _has_field(rows, f'{joints[0]}_q_cmd'):
        fig, axes = plt.subplots(n, 1, figsize=(10, 2.5 * n), sharex=True)
        if n == 1:
            axes = [axes]
        for ax, j in zip(axes, joints):
            q_cmd = np.array([float(r.get(f'{j}_q_cmd', np.nan)) for r in rows], dtype=float)
            if f'{j}_q' in fieldnames:
                q = np.array([float(r[f'{j}_q']) for r in rows], dtype=float)
                ax.plot(t, q, label='q')
            ax.plot(t, q_cmd, label='q_cmd')
            ax.set_ylabel(j)
            ax.legend(loc='upper right')
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel('time [s]')
        fig.suptitle('Joint position command')
        fig.tight_layout()
        fig.savefig(out_p / 'joint_position_command.png', dpi=150)
        plt.show()
        plt.close(fig)

    if _has_field(rows, f'{joints[0]}_dq_cmd'):
        fig, axes = plt.subplots(n, 1, figsize=(10, 2.5 * n), sharex=True)
        if n == 1:
            axes = [axes]
        for ax, j in zip(axes, joints):
            dq_cmd = np.array([float(r.get(f'{j}_dq_cmd', np.nan)) for r in rows], dtype=float)
            if f'{j}_dq' in fieldnames:
                dq = np.array([float(r[f'{j}_dq']) for r in rows], dtype=float)
                ax.plot(t, dq, label='dq')
            ax.plot(t, dq_cmd, label='dq_cmd')
            ax.set_ylabel(j)
            ax.legend(loc='upper right')
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel('time [s]')
        fig.suptitle('Joint velocity command')
        fig.tight_layout()
        fig.savefig(out_p / 'joint_velocity_command.png', dpi=150)
        plt.show()
        plt.close(fig)

    if _has_field(rows, f'{joints[0]}_tau_cmd_in'):
        fig, axes = plt.subplots(n, 1, figsize=(10, 2.5 * n), sharex=True)
        if n == 1:
            axes = [axes]
        for ax, j in zip(axes, joints):
            tau_cmd_in = np.array([float(r.get(f'{j}_tau_cmd_in', np.nan)) for r in rows], dtype=float)
            if f'{j}_tau_meas' in fieldnames:
                tau_meas = np.array([float(r[f'{j}_tau_meas']) for r in rows], dtype=float)
                ax.plot(t, tau_meas, label='tau_meas')
            elif f'{j}_tau_cmd' in fieldnames:
                tau = np.array([float(r[f'{j}_tau_cmd']) for r in rows], dtype=float)
                ax.plot(t, tau, label='tau_cmd')
            ax.plot(t, tau_cmd_in, label='tau_cmd_in')
            ax.set_ylabel(j)
            ax.legend(loc='upper right')
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel('time [s]')
        fig.suptitle('Joint effort command')
        fig.tight_layout()
        fig.savefig(out_p / 'joint_effort_command.png', dpi=150)
        plt.show()
        plt.close(fig)

    if has_q:
        rms = []
        for j in joints:
            q = np.array([float(r[f'{j}_q']) for r in rows])
            qd = np.array([float(r[f'{j}_q_des']) for r in rows])
            rms.append(float(np.sqrt(np.mean((qd - q) ** 2))))
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.bar(joints, rms)
        ax.set_ylabel('RMS |q_des - q|')
        ax.set_title('Position tracking error (RMS)')
        ax.grid(True, axis='y', alpha=0.3)
        fig.tight_layout()
        fig.savefig(out_p / 'tracking_error_summary.png', dpi=150)
        plt.show()
        plt.close(fig)

    need = {'ee_x', 'ee_y', 'ee_z', 'ee_des_x', 'ee_des_y', 'ee_des_z'}
    if need.issubset(set(fieldnames)):
        ex = np.array([float(r['ee_x']) for r in rows])
        ey = np.array([float(r['ee_y']) for r in rows])
        ez = np.array([float(r['ee_z']) for r in rows])
        dx = np.array([float(r['ee_des_x']) for r in rows])
        dy = np.array([float(r['ee_des_y']) for r in rows])
        dz = np.array([float(r['ee_des_z']) for r in rows])
        fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
        labels = [('x', ex, dx), ('y', ey, dy), ('z', ez, dz)]
        for ax, (lab, a, d) in zip(axes, labels):
            ax.plot(t, a, label=f'{lab} actual')
            ax.plot(t, d, label=f'{lab} des')
            ax.set_ylabel(lab)
            ax.legend(loc='upper right')
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel('time [s]')
        fig.suptitle('Task-space position vs desired')
        fig.tight_layout()
        fig.savefig(out_p / 'task_space_xyz_vs_desired.png', dpi=150)
        plt.show()
        plt.close(fig)


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
    args = p.parse_args()
    csv_path = Path(args.csv)
    out = Path(args.output_dir) if args.output_dir else csv_path.parent / 'plots'
    generate_plots(str(csv_path), str(out))
    print(f'Plots written to {out}')


if __name__ == '__main__':
    main()
