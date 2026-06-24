"""Recording session: two CSVs sharing one t0.

  <name>_emg.csv    t [s], one column per EMG channel       (t = k / fs)
  <name>_joints.csv t [s], pos_*, vel_*, eff_* per joint    (t = arrival - t0)

t0 is the monotonic instant DelsysAPI Start() returned, so EMG sample 0 and
joint rows share the same origin (software sync, ~10 ms accuracy — there is no
hardware trigger between the Trigno base and the exo).
"""
import csv
import datetime
import os

import config


class RecordingSession:
    def __init__(self, out_dir, name, emg_channels, joint_names, side, mode_str):
        os.makedirs(out_dir, exist_ok=True)
        self.emg_path = os.path.join(out_dir, f"{name}_emg.csv")
        self.joints_path = os.path.join(out_dir, f"{name}_joints.csv")
        for p in (self.emg_path, self.joints_path):
            if os.path.exists(p):
                raise FileExistsError(f"{p} already exists - pick another name")

        self.fs = emg_channels[0].sample_rate
        self.n_emg = len(emg_channels)
        self.sample_index = 0
        # Per-channel samples not yet written (channels drain unevenly; rows are
        # only written once every channel has a sample for that row).
        self._emg_residual = [[] for _ in emg_channels]

        started = datetime.datetime.now().isoformat(timespec="seconds")
        meta = f"# started={started} arm={side} mode=\"{mode_str}\""

        self._emg_file = open(self.emg_path, "w", newline="")
        self._emg = csv.writer(self._emg_file)
        self._emg_file.write(meta + f" emg_rate_hz={self.fs}\n")
        self._emg.writerow(
            ["t"] + [f"{ch.sensor_name}|{ch.name}".replace(" ", "_") for ch in emg_channels]
        )

        self._joints_file = open(self.joints_path, "w", newline="")
        self._joints = csv.writer(self._joints_file)
        self._joints_file.write(meta + f" encoder_rate_hz={config.ENCODER_RATE_HZ}\n")
        self._joints.writerow(
            ["t"]
            + [f"pos_{j}" for j in joint_names]
            + [f"vel_{j}" for j in joint_names]
            + [f"eff_{j}" for j in joint_names]
        )

    def append_emg(self, chunks):
        """chunks: per-channel lists of new samples (from DelsysClient.drain())."""
        for ci, chunk in enumerate(chunks):
            self._emg_residual[ci].extend(chunk)
        n = min(len(r) for r in self._emg_residual)
        if n == 0:
            return
        for k in range(n):
            t = (self.sample_index + k) / self.fs
            self._emg.writerow([f"{t:.6f}"] + [self._emg_residual[ci][k] for ci in range(self.n_emg)])
        self.sample_index += n
        self._emg_residual = [r[n:] for r in self._emg_residual]

    def append_joints(self, rows):
        """rows: [t, pos..., vel..., eff...] (from JointStateRecorder.drain())."""
        for row in rows:
            self._joints.writerow([f"{row[0]:.6f}"] + row[1:])

    def emg_seconds(self):
        return self.sample_index / self.fs

    def close(self):
        # Flush whatever is complete; drop the (sub-millisecond) ragged tail.
        self._emg_file.close()
        self._joints_file.close()
