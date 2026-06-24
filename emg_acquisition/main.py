"""EMG (Delsys Trigno) + exo encoder acquisition GUI.

Run via ./run.sh (needs ROS sourced, Delsys native libs on LD_LIBRARY_PATH,
dialout group). Workflow: launch stack -> connect -> scan -> start.
"""
import datetime
import sys
import time
from collections import deque

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QApplication, QComboBox, QDoubleSpinBox, QFormLayout, QGroupBox,
    QHBoxLayout, QLabel, QLineEdit, QMainWindow, QMessageBox, QPushButton,
    QVBoxLayout, QWidget,
)

import config
from delsys_client import DelsysClient, parse_mode_emg_rate
from recorder import RecordingSession
from ros_bridge import RosBridge, StackLauncher
from windowing import process_recording

PLOT_WINDOW_S = 5.0     # seconds of history shown in the live plots
GUI_PERIOD_MS = 33      # drain/plot timer (~30 Hz)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("EMG + Encoder Acquisition")
        self.client = DelsysClient()
        self.stack = StackLauncher()
        self.bridge = RosBridge()
        self.session = None
        self.recording = False
        self.previewing = False
        self.duration_s = 0.0
        self.emg_plot_bufs = []   # per channel: (deque of (t, sample), curve)
        self.emg_sample_count = []
        self._emg_t0 = None
        self._joint_names_set = False
        self._build_ui()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(GUI_PERIOD_MS)

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        controls = QVBoxLayout()

        ros_box = QGroupBox("Exo stack (read-only)")
        ros_form = QFormLayout(ros_box)
        self.side_combo = QComboBox()
        self.side_combo.addItems(["right", "left"])
        self.stack_btn = QPushButton("Launch stack")
        self.stack_btn.clicked.connect(self._toggle_stack)
        self.stack_status = QLabel("not running")
        ros_form.addRow("Arm", self.side_combo)
        ros_form.addRow(self.stack_btn)
        ros_form.addRow("Encoders", self.stack_status)
        controls.addWidget(ros_box)

        delsys_box = QGroupBox("Delsys Trigno")
        delsys_form = QFormLayout(delsys_box)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self._connect_delsys)
        self.scan_btn = QPushButton("Scan sensors")
        self.scan_btn.clicked.connect(self._scan)
        self.scan_btn.setEnabled(False)
        self.mode_combo = QComboBox()
        self.mode_combo.setEnabled(False)
        self.mode_combo.currentTextChanged.connect(self._mode_changed)
        self.delsys_status = QLabel("disconnected")
        delsys_form.addRow(self.connect_btn)
        delsys_form.addRow(self.scan_btn)
        delsys_form.addRow("Mode", self.mode_combo)
        delsys_form.addRow("Status", self.delsys_status)
        controls.addWidget(delsys_box)

        id_box = QGroupBox("Identify (tap to find)")
        id_form = QFormLayout(id_box)
        self.preview_btn = QPushButton("Start preview")
        self.preview_btn.setEnabled(False)
        self.preview_btn.clicked.connect(self._toggle_preview)
        self.preview_status = QLabel(
            "Start preview, then tap each sensor — the plot row that spikes is that sensor."
        )
        self.preview_status.setWordWrap(True)
        id_form.addRow(self.preview_btn)
        id_form.addRow(self.preview_status)
        controls.addWidget(id_box)

        rec_box = QGroupBox("Recording")
        rec_form = QFormLayout(rec_box)
        self.name_edit = QLineEdit(datetime.datetime.now().strftime("session_%Y%m%d_%H%M%S"))
        self.duration_spin = QDoubleSpinBox()
        self.duration_spin.setRange(0.0, 3600.0)
        self.duration_spin.setDecimals(1)
        self.duration_spin.setSuffix(" s")
        self.duration_spin.setSpecialValueText("manual stop")
        self.start_btn = QPushButton("Start")
        self.start_btn.clicked.connect(self._start_recording)
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.clicked.connect(self._stop_recording)
        self.stop_btn.setEnabled(False)
        self.rec_status = QLabel("idle")
        rec_form.addRow("File name", self.name_edit)
        rec_form.addRow("Duration", self.duration_spin)
        btns = QHBoxLayout()
        btns.addWidget(self.start_btn)
        btns.addWidget(self.stop_btn)
        rec_form.addRow(btns)
        rec_form.addRow("Status", self.rec_status)
        controls.addWidget(rec_box)
        controls.addStretch()

        self.plots = pg.GraphicsLayoutWidget()
        self.joint_plot = None
        self.joint_curves = []
        self._rebuild_plots()

        central = QWidget()
        layout = QHBoxLayout(central)
        left = QWidget()
        left.setLayout(controls)
        left.setFixedWidth(320)
        layout.addWidget(left)
        layout.addWidget(self.plots, stretch=1)
        self.setCentralWidget(central)
        self.resize(1200, 700)

    def _rebuild_plots(self, joint_names=None):
        """One row for the 3 joints + one row per known EMG channel.

        Called at startup, when joint names first arrive, and after Scan/mode
        change (when the EMG channel list changes). All rows share a rolling
        "seconds ago" x-axis [-PLOT_WINDOW_S, 0].
        """
        self.plots.clear()
        self.joint_curves = []
        self.emg_plot_bufs = []
        self.emg_sample_count = []

        self.joint_plot = self.plots.addPlot(title="Joint angles [rad]")
        self.joint_plot.addLegend()
        self.joint_plot.showGrid(x=True, y=True, alpha=0.3)
        self.joint_plot.setXRange(-PLOT_WINDOW_S, 0)
        names = joint_names or [f"joint_{i + 1}" for i in range(3)]
        for j, name in enumerate(names):
            self.joint_curves.append(
                self.joint_plot.plot(pen=pg.mkPen(pg.intColor(j, max(3, len(names))), width=2),
                                     name=name))

        for ch in self.client.emg_channels:
            self.plots.nextRow()
            # Pair number is printed on the Avanti housing; show it large+bold so
            # the user can match plot row to physical sensor at a glance.
            title = (
                f"<span style='font-size:14pt; font-weight:bold; color:#ffc107'>"
                f"Pair #{ch.pair_number}</span>"
                f"  —  {ch.sensor_name} {ch.name} [mV] ({ch.sample_rate:.0f} Hz)"
            )
            p = self.plots.addPlot(title=title)
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setXRange(-PLOT_WINDOW_S, 0)
            curve = p.plot(pen=pg.mkPen(width=1))
            n = int(PLOT_WINDOW_S * ch.sample_rate)
            self.emg_plot_bufs.append((deque(maxlen=n), curve))
            self.emg_sample_count.append(0)

    # ------------------------------------------------------------ callbacks
    def _toggle_stack(self):
        if self.stack.running():
            self.bridge.stop()
            self.stack.stop()
            self.stack_btn.setText("Launch stack")
            self.stack_status.setText("not running")
            return
        side = self.side_combo.currentText()
        try:
            self.stack.start(side)
            self.bridge.start(side)
        except Exception as e:
            QMessageBox.critical(self, "ROS stack", str(e))
            return
        self.stack_btn.setText("Stop stack")
        self.side_combo.setEnabled(False)

    def _connect_delsys(self):
        self.delsys_status.setText("connecting...")
        QApplication.processEvents()
        try:
            station = self.client.connect()
        except Exception as e:
            self.delsys_status.setText("connect failed")
            QMessageBox.critical(self, "Delsys", str(e))
            return
        self.delsys_status.setText(f"connected ({station})")
        self.connect_btn.setEnabled(False)
        self.scan_btn.setEnabled(True)

    def _scan(self):
        self.delsys_status.setText("scanning...")
        QApplication.processEvents()
        try:
            sensors = self.client.scan()
            if not sensors:
                self.delsys_status.setText("no sensors found")
                return
            mode, rate = self.client.auto_select_mode()
        except Exception as e:
            self.delsys_status.setText("scan failed")
            QMessageBox.critical(self, "Delsys", str(e))
            return
        try:
            # Configure now (not at Start) so the EMG channel list is known and
            # the plot rows for every channel appear right away.
            self.client.configure()
            self._rebuild_plots(self._current_joint_names())
            self.preview_btn.setEnabled(True)
        except Exception as e:
            QMessageBox.critical(self, "Delsys", f"Configure failed: {e}")
            return
        self.delsys_status.setText(
            f"{len(sensors)} sensor(s), {len(self.client.emg_channels)} EMG ch"
            + (f" @ {rate:.0f} Hz" if rate else "")
        )
        # Mode dropdown: sensor 0's EMG-rate modes, applied to all sensors.
        self.mode_combo.blockSignals(True)
        self.mode_combo.clear()
        for m in self.client.available_modes(0):
            if parse_mode_emg_rate(m) is not None:
                self.mode_combo.addItem(m)
        if mode is not None:
            self.mode_combo.setCurrentText(mode)
        self.mode_combo.blockSignals(False)
        self.mode_combo.setEnabled(True)

    def _mode_changed(self, mode):
        if not mode or self.recording:
            return
        try:
            for i in range(self.client.sensor_count()):
                self.client.set_mode(i, mode)
            self.client.configure()  # channel list/rates change with the mode
            self._rebuild_plots(self._current_joint_names())
            self.preview_btn.setEnabled(True)
            rate = parse_mode_emg_rate(mode)
            self.delsys_status.setText(f"mode set; EMG @ {rate:.0f} Hz" if rate else "mode set")
        except Exception as e:
            QMessageBox.critical(self, "Delsys", str(e))

    def _toggle_preview(self):
        if self.previewing:
            self._stop_preview()
        else:
            self._start_preview()

    def _start_preview(self):
        if self.recording:
            QMessageBox.warning(self, "Preview", "Stop recording before previewing.")
            return
        if not self.client.connected or self.client.sensor_count() == 0:
            QMessageBox.warning(self, "Preview", "Connect to the Delsys base and scan sensors first.")
            return
        if self.client.pipeline_state() != "Armed":
            try:
                self.client.configure()
                self._rebuild_plots(self._current_joint_names())
            except Exception as e:
                QMessageBox.critical(self, "Preview", f"Pipeline not ready: {e}")
                return
        try:
            t0 = self.client.start()
        except Exception as e:
            QMessageBox.critical(self, "Preview", str(e))
            return
        self._emg_t0 = t0
        for buf, _ in self.emg_plot_bufs:
            buf.clear()
        self.emg_sample_count = [0] * len(self.emg_plot_bufs)
        self.previewing = True
        self.preview_btn.setText("Stop preview")
        self.preview_status.setText("Streaming live. Tap a sensor — the plot row that spikes is it.")
        # Block conflicting actions while previewing.
        self.start_btn.setEnabled(False)
        self.scan_btn.setEnabled(False)
        self.mode_combo.setEnabled(False)

    def _stop_preview(self):
        if not self.previewing:
            return
        try:
            self.client.stop()
        except Exception as e:
            QMessageBox.warning(self, "Preview", f"Stop error: {e}")
        self.previewing = False
        self.preview_btn.setText("Start preview")
        self.preview_status.setText(
            "Start preview, then tap each sensor — the plot row that spikes is that sensor."
        )
        self.start_btn.setEnabled(True)
        self.scan_btn.setEnabled(True)
        if self.mode_combo.count() > 0:
            self.mode_combo.setEnabled(True)

    def _current_joint_names(self):
        rec = self.bridge.recorder
        return rec.joint_names if (rec is not None and rec.joint_names) else None

    def _start_recording(self):
        if self.recording:
            return
        if not self.client.connected or self.client.sensor_count() == 0:
            QMessageBox.warning(self, "Start", "Connect to the Delsys base and scan sensors first.")
            return
        rec = self.bridge.recorder
        if rec is None or not rec.alive():
            QMessageBox.warning(
                self, "Start",
                "No encoder data: launch the exo stack and wait for joint states.")
            return
        try:
            # Scan already configured the pipeline; reconfigure only if it was
            # since reset (e.g. a failed previous run).
            if self.client.pipeline_state() != "Armed":
                self.client.configure()
                self._rebuild_plots(self._current_joint_names())
            session = RecordingSession(
                out_dir=config.DEFAULT_OUTPUT_DIR,
                name=self.name_edit.text().strip(),
                emg_channels=self.client.emg_channels,
                joint_names=rec.joint_names,
                side=self.side_combo.currentText(),
                mode_str=self.mode_combo.currentText(),
            )
        except Exception as e:
            QMessageBox.critical(self, "Start", str(e))
            return

        self.session = session
        self.duration_s = self.duration_spin.value()
        t0 = self.client.start()
        self._emg_t0 = t0
        for buf, _ in self.emg_plot_bufs:
            buf.clear()
        self.emg_sample_count = [0] * len(self.emg_plot_bufs)
        rec.start_recording(t0)
        self._wall_start = time.monotonic()
        self.recording = True
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.name_edit.setEnabled(False)

    def _stop_recording(self):
        if not self.recording:
            return
        self.recording = False
        rec = self.bridge.recorder
        try:
            self.client.stop()
        except Exception as e:
            print(f"Delsys stop error: {e}")
        if rec is not None:
            rec.stop_recording()
        # Final drain so the tail of both streams lands in the files.
        self._drain_to_session()
        emg_s = self.session.emg_seconds()
        emg_path = self.session.emg_path
        joints_path = self.session.joints_path
        paths = f"{emg_path}\n{joints_path}"
        self.session.close()
        self.session = None
        self.rec_status.setText(f"saved {emg_s:.1f} s")

        # Build the windowed .npy dataset from the just-saved CSVs.
        if config.AUTO_WINDOW_ON_STOP:
            try:
                info = process_recording(emg_path, joints_path)
                paths += (f"\n\nDataset: {info['x_shape']} windows "
                          f"({info['n_emg']} EMG + {info['n_joint']} joints)\n"
                          f"{info['x_path']}")
                if info["y_path"]:
                    paths += f"\n{info['y_path']}"
            except Exception as e:
                paths += f"\n\nWindowing skipped: {e}"
        QMessageBox.information(self, "Recording saved", paths)
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.name_edit.setEnabled(True)
        self.name_edit.setText(datetime.datetime.now().strftime("session_%Y%m%d_%H%M%S"))

    # ---------------------------------------------------------------- timer
    def _drain_to_session(self):
        emg_chunks = self.client.drain() if self.client.streaming or self.recording else None
        if emg_chunks:
            if self.session:
                self.session.append_emg(emg_chunks)
            for ci, chunk in enumerate(emg_chunks):
                if ci >= len(self.emg_plot_bufs):
                    break
                fs = self.client.emg_channels[ci].sample_rate
                base = self.emg_sample_count[ci]
                buf = self.emg_plot_bufs[ci][0]
                for k, v in enumerate(chunk):
                    buf.append((self._emg_t0 + (base + k) / fs, v))
                self.emg_sample_count[ci] += len(chunk)
        rec = self.bridge.recorder
        if rec is not None and self.session:
            self.session.append_joints(rec.drain())

    def _tick(self):
        rec = self.bridge.recorder
        if self.stack.running():
            self.stack_status.setText("alive" if (rec and rec.alive()) else "no joint states")
        elif self.stack.proc is not None:
            self.stack_status.setText("stack exited!")

        # Swap in the real joint names once known (keeps EMG plots intact only
        # by rebuilding outside of a recording).
        if (rec is not None and rec.joint_names and not self._joint_names_set
                and not self.recording):
            self._joint_names_set = True
            self._rebuild_plots(rec.joint_names)

        if self.recording or self.previewing:
            self._drain_to_session()

        # All curves share a rolling "seconds ago" x-axis.
        now = time.monotonic()
        if rec is not None and rec.plot_buf and self.joint_curves:
            snap = list(rec.plot_buf)
            x = np.fromiter((t - now for t, _ in snap), dtype=float, count=len(snap))
            for j, curve in enumerate(self.joint_curves):
                y = np.fromiter((p[j] for _, p in snap if j < len(p)), dtype=float)
                curve.setData(x[: len(y)], y)
        for buf, curve in self.emg_plot_bufs:
            if buf:
                snap = list(buf)
                stride = max(1, len(snap) // 2500)  # cap points per redraw
                snap = snap[::stride]
                # Anchor at the newest sample (not wall clock): sample times are
                # nominal (k/fs) and drift slightly vs the monotonic clock.
                ref = buf[-1][0]
                x = np.fromiter((t - ref for t, _ in snap), dtype=float, count=len(snap))
                y = np.fromiter((v for _, v in snap), dtype=float, count=len(snap))
                curve.setData(x, y)

        if not self.recording:
            return
        elapsed = time.monotonic() - self._wall_start
        self.rec_status.setText(f"recording {elapsed:.1f} s ({self.session.emg_seconds():.1f} s EMG)")
        if self.duration_s > 0 and elapsed >= self.duration_s:
            self._stop_recording()

    def closeEvent(self, event):
        if self.previewing:
            self._stop_preview()
        if self.recording:
            self._stop_recording()
        self.bridge.stop()
        self.stack.stop()
        event.accept()


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
