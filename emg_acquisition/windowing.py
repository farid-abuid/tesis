"""Replicates the Sedighi et al. (RAL 2023) 30x6 CNN-LSTM input windowing.

The model in that paper does NOT consume raw sensor samples. EMG (2 kHz) and
joint angles (their encoder: 50 Hz; ours: ~500 Hz) arrive at different rates, so
both streams are first resampled onto ONE common time grid and only then sliced
into fixed [rows x channels] windows:

    grid step   = 1 / grid_hz           (paper: 2 ms  -> 500 Hz)
    window rows = window_ms * grid_hz    (paper: 60 ms -> 30 rows)
    channels    = n_emg + n_joint        (paper: 3 + 3 -> 6)   => 30 x 6
    slide step  = step_rows grid samples (paper: 1 row = 2 ms)

The fast EMG is low-pass filtered (the paper uses a 2nd-order Butterworth at
8 Hz, i.e. it works on the EMG *envelope*) and then decimated to the grid; the
slow joint stream is linearly interpolated up to the grid. Both are Z-scored.
A training target is the joint-angle vector `horizon_ms` into the future
(paper: 450 ms = 225 rows). Set horizon_ms=0 to get input-only windows.

Our recordings differ from the paper's hardware (see config.py): EMG 2148 Hz,
joints ~500 Hz, so the EMG/joint rate ratio is ~4:1 instead of ~40:1 -- the same
machinery, far less up-sampling of the slow channel.
"""
from __future__ import annotations

import csv
import os
from dataclasses import dataclass, field

import numpy as np
from scipy.signal import butter, filtfilt

import config


@dataclass
class WindowConfig:
    grid_hz: float = config.WINDOW_GRID_HZ          # common resampling grid (paper: 2 ms / 500 Hz)
    window_ms: float = config.WINDOW_MS             # window length (paper: 60 ms -> 30 rows)
    step_rows: int = 1                              # slide between windows, in grid rows (paper: 1 = 2 ms)
    emg_lpf_hz: float = 8.0                         # EMG envelope low-pass (paper: 2nd-order Butterworth)
    emg_lpf_order: int = 2
    horizon_ms: float = config.PREDICTION_HORIZON_MS  # target = joints this far ahead (paper: 450 ms)
    zscore: bool = True

    @property
    def window_rows(self) -> int:
        return int(round(self.window_ms / 1000.0 * self.grid_hz))

    @property
    def horizon_rows(self) -> int:
        return int(round(self.horizon_ms / 1000.0 * self.grid_hz))


@dataclass
class Recording:
    t_emg: np.ndarray            # [N_emg]   seconds
    emg: np.ndarray              # [N_emg, C_emg]
    emg_names: list
    t_joint: np.ndarray          # [N_joint] seconds
    pos: np.ndarray              # [N_joint, C_joint]  joint angles only
    joint_names: list
    emg_rate_hz: float = field(default=float("nan"))


def _read_csv(path):
    """Returns (header[list], data[ndarray]); skips a leading '# ...' comment line."""
    with open(path) as f:
        first = f.readline()
        if not first.startswith("#"):
            f.seek(0)
        reader = csv.reader(f)
        header = next(reader)
        rows = [[float(x) for x in r] for r in reader if r]
    return header, np.asarray(rows, dtype=float)


def load_recording(emg_csv: str, joints_csv: str) -> Recording:
    """Load an emg/joints CSV pair written by recorder.RecordingSession."""
    emg_hdr, emg_data = _read_csv(emg_csv)
    j_hdr, j_data = _read_csv(joints_csv)

    emg_names = emg_hdr[1:]
    t_emg, emg = emg_data[:, 0], emg_data[:, 1:]

    # Joints CSV is t, pos_*, vel_*, eff_*; the model input uses positions only.
    pos_idx = [i for i, c in enumerate(j_hdr) if c.startswith("pos_")]
    joint_names = [j_hdr[i] for i in pos_idx]
    t_joint, pos = j_data[:, 0], j_data[:, pos_idx]

    emg_rate = float("nan")
    if len(t_emg) > 1:
        emg_rate = 1.0 / np.median(np.diff(t_emg))
    return Recording(t_emg, emg, emg_names, t_joint, pos, joint_names, emg_rate)


def _butter_lpf(x: np.ndarray, fs: float, cutoff: float, order: int) -> np.ndarray:
    """Zero-phase low-pass, per column. Falls back to passthrough if too short."""
    nyq = 0.5 * fs
    wn = min(cutoff / nyq, 0.99)
    b, a = butter(order, wn, btype="low")
    padlen = 3 * max(len(a), len(b))
    if x.shape[0] <= padlen:
        return x
    return filtfilt(b, a, x, axis=0)


def _resample(t_src: np.ndarray, x_src: np.ndarray, t_grid: np.ndarray) -> np.ndarray:
    """Linear interpolation of each column of x_src onto t_grid."""
    out = np.empty((t_grid.shape[0], x_src.shape[1]), dtype=float)
    for c in range(x_src.shape[1]):
        out[:, c] = np.interp(t_grid, t_src, x_src[:, c])
    return out


def _zscore(x: np.ndarray) -> np.ndarray:
    mu = x.mean(axis=0, keepdims=True)
    sd = x.std(axis=0, keepdims=True)
    sd[sd == 0] = 1.0
    return (x - mu) / sd


def build_matrix(rec: Recording, cfg: WindowConfig):
    """Resample both streams to the common grid -> (t_grid, M[T, n_emg+n_joint]).

    Columns are [EMG..., joint angle...], matching the paper's 30 x 6 layout.
    """
    t0 = max(rec.t_emg[0], rec.t_joint[0])
    t1 = min(rec.t_emg[-1], rec.t_joint[-1])
    n = int(np.floor((t1 - t0) * cfg.grid_hz)) + 1
    t_grid = t0 + np.arange(n) / cfg.grid_hz

    emg = rec.emg
    if cfg.emg_lpf_hz and np.isfinite(rec.emg_rate_hz):
        emg = _butter_lpf(emg, rec.emg_rate_hz, cfg.emg_lpf_hz, cfg.emg_lpf_order)

    emg_g = _resample(rec.t_emg, emg, t_grid)        # decimate fast EMG to grid
    pos_g = _resample(rec.t_joint, rec.pos, t_grid)  # interp slow joints up to grid

    M = np.hstack([emg_g, pos_g])
    if cfg.zscore:
        M = _zscore(M)
    return t_grid, M


def make_windows(rec: Recording, cfg: WindowConfig):
    """Build sliding windows (and optional future targets) from a recording.

    Returns (X, Y) where
        X: [n_windows, window_rows, n_channels]   (paper: [-, 30, 6])
        Y: [n_windows, n_joint] joint angles horizon_rows ahead, or None.
    """
    _, M = build_matrix(rec, cfg)
    w = cfg.window_rows
    h = cfg.horizon_rows
    n_joint = rec.pos.shape[1]
    j0 = M.shape[1] - n_joint  # joint columns are the trailing block

    last_start = M.shape[0] - w - h
    if last_start < 0:
        raise ValueError(
            f"recording too short: have {M.shape[0]} grid rows, "
            f"need >= {w + h} (window {w} + horizon {h})"
        )

    starts = range(0, last_start + 1, cfg.step_rows)
    X = np.stack([M[s:s + w] for s in starts])
    Y = None
    if h > 0:
        Y = np.stack([M[s + w - 1 + h, j0:] for s in starts])
    return X, Y


def process_recording(emg_csv, joints_csv, cfg=None, out_dir=None):
    """Window one recording and save it as .npy. Used by the GUI on stop.

    Returns a dict with the output paths and the X/Y shapes; raises on a
    recording too short to yield a single window (callers may want to warn).
    """
    cfg = cfg or WindowConfig()
    out_dir = out_dir or config.DATASET_DIR

    rec = load_recording(emg_csv, joints_csv)
    X, Y = make_windows(rec, cfg)

    os.makedirs(out_dir, exist_ok=True)
    base = os.path.basename(emg_csv).replace("_emg.csv", "")
    x_path = os.path.join(out_dir, f"{base}_X.npy")
    y_path = os.path.join(out_dir, f"{base}_Y.npy")
    np.save(x_path, X)
    if Y is not None:
        np.save(y_path, Y)
    return {
        "x_path": x_path,
        "y_path": y_path if Y is not None else None,
        "x_shape": X.shape,
        "y_shape": None if Y is None else Y.shape,
        "n_emg": rec.emg.shape[1],
        "n_joint": rec.pos.shape[1],
    }


if __name__ == "__main__":
    import glob

    here = config.DEFAULT_OUTPUT_DIR
    emg_csvs = sorted(glob.glob(os.path.join(here, "*_emg.csv")))
    if not emg_csvs:
        raise SystemExit(f"no recordings in {here}")
    emg_csv = emg_csvs[-1]
    joints_csv = emg_csv.replace("_emg.csv", "_joints.csv")

    rec = load_recording(emg_csv, joints_csv)
    print(f"recording : {os.path.basename(emg_csv)}")
    print(f"EMG       : {rec.emg.shape[0]} samples @ ~{rec.emg_rate_hz:.1f} Hz, "
          f"{rec.emg.shape[1]} ch {rec.emg_names}")
    jr = 1.0 / np.median(np.diff(rec.t_joint))
    print(f"joints    : {rec.pos.shape[0]} samples @ ~{jr:.1f} Hz, "
          f"{rec.pos.shape[1]} ch {rec.joint_names}")

    cfg = WindowConfig()
    print(f"\ncfg       : grid {cfg.grid_hz:.0f} Hz, window {cfg.window_ms:.0f} ms "
          f"-> {cfg.window_rows} rows, horizon {cfg.horizon_ms:.0f} ms "
          f"-> {cfg.horizon_rows} rows")

    info = process_recording(emg_csv, joints_csv, cfg)
    print(f"\nX (windows): {info['x_shape']}   # [n_windows, rows, channels]")
    print(f"Y (targets): {info['y_shape']}")
    print(f"\nsaved     : {info['x_path']}")
    if info["y_path"]:
        print(f"            {info['y_path']}")
