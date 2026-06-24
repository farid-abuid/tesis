"""Slim Delsys Trigno client for EMG acquisition.

Loads the .NET DelsysAPI from the vendor Example-Applications folder (path in
config.py) instead of copying it here; the key/license strings are parsed out
of the vendor's AeroPy/TrignoBase.py at runtime so no credentials live in this
repo.
"""
import os
import re
import sys
import threading
import time
from dataclasses import dataclass, field

import config


def _load_aeropy():
    # The DelsysAPI's RF-subsystem discovery loads its native USB helpers
    # (TrignoCentro.so, libSiUSBXp.so, CentroUSB_*.dll, ...) relative to the
    # process working directory — the vendor demo only works because its run.sh
    # cds into the Python/ folder first. Mirror that here; the rest of the app
    # uses absolute paths, so the cwd change is safe.
    if not os.path.isdir(config.DELSYS_PYTHON_DIR):
        raise FileNotFoundError(f"Delsys dir not found: {config.DELSYS_PYTHON_DIR}")
    os.chdir(config.DELSYS_PYTHON_DIR)

    from pythonnet import load
    load("coreclr")
    import clr
    resources = os.path.join(config.DELSYS_PYTHON_DIR, "resources")
    sys.path.append(resources)
    clr.AddReference("DelsysAPI")
    clr.AddReference("System.Collections")
    from Aero import AeroPy  # noqa: E402  (available only after AddReference)
    return AeroPy


def _load_credentials():
    """Parse key/license from the vendor TrignoBase.py (kept out of this repo)."""
    path = os.path.join(config.DELSYS_PYTHON_DIR, "AeroPy", "TrignoBase.py")
    text = open(path).read()
    key = re.search(r'^key\s*=\s*"(.+?)"', text, re.M)
    lic = re.search(r'^license\s*=\s*"(.+?)"', text, re.M)
    if not key or not lic:
        raise RuntimeError(f"Could not parse key/license from {path}")
    return key.group(1), lic.group(1)


# Matches the EMG rate inside a sample-mode string, e.g. "EMG raw (2148Hz), ..."
_MODE_EMG_RATE = re.compile(r"EMG[^,(]*\(\s*(\d+(?:[.,]\d+)?)\s*k?Hz", re.IGNORECASE)


def parse_mode_emg_rate(mode: str):
    """EMG rate [Hz] parsed from a sample-mode string, or None."""
    m = _MODE_EMG_RATE.search(mode)
    if not m:
        return None
    rate = float(m.group(1).replace(",", "."))
    if "khz" in mode[m.start():m.end() + 3].lower():
        rate *= 1000.0
    return rate


@dataclass
class EmgChannel:
    sensor_index: int
    sensor_name: str
    name: str
    guid: str
    sample_rate: float  # Hz
    pair_number: int = 0  # printed on the Avanti housing, for physical identification


@dataclass
class DelsysClient:
    AeroPy: object = None
    base: object = None
    connected: bool = False
    streaming: bool = False
    emg_channels: list = field(default_factory=list)
    # Per-channel sample chunks pending consumption (guarded by _lock).
    _pending: list = field(default_factory=list)
    _lock: threading.Lock = field(default_factory=threading.Lock)
    _poll_thread: threading.Thread = None
    _station_type: str = ""

    def connect(self):
        if self.AeroPy is None:
            self.AeroPy = _load_aeropy()
            self.base = self.AeroPy()
        key, lic = _load_credentials()
        self.base.ValidateBase(key, lic)
        self._station_type = self.base.GetTrignoReceiverType()
        self.connected = True
        return self._station_type

    def pipeline_state(self):
        return str(self.base.GetPipelineState()) if self.base else "Disconnected"

    def scan(self):
        """Scan for paired sensors; returns list of (name, mode) per sensor."""
        if self.pipeline_state() == "Armed":
            self.base.ResetPipeline()
        self.base.ScanSensors().Result
        self.base.SelectAllSensors()
        sensors = self.base.GetSensors()
        return [
            (str(sensors[i].FriendlyName), str(self.base.GetCurrentSensorMode(i)))
            for i in range(len(sensors))
        ]

    def sensor_count(self):
        return len(self.base.GetSensors()) if self.connected else 0

    def available_modes(self, sensor_index):
        return [str(m) for m in self.base.AvailableSensorModes(sensor_index)]

    def set_mode(self, sensor_index, mode):
        self.base.SetSampleMode(sensor_index, mode)

    def auto_select_mode(self, target_hz=None):
        """Set every sensor to its EMG mode closest to target_hz.

        Returns (chosen_mode, parsed_rate) or (None, None) when no mode string
        contains a parseable EMG rate (mode left unchanged).
        """
        target_hz = target_hz or config.TARGET_EMG_RATE_HZ
        chosen = (None, None)
        for i in range(self.sensor_count()):
            best = None
            for mode in self.available_modes(i):
                rate = parse_mode_emg_rate(mode)
                if rate is None:
                    continue
                if best is None or abs(rate - target_hz) < abs(best[1] - target_hz):
                    best = (mode, rate)
            if best is not None:
                self.set_mode(i, best[0])
                chosen = best
        return chosen

    def configure(self):
        """Configure the pipeline and collect the EMG channel list (EMG-only output)."""
        if self.pipeline_state() == "Armed":
            self.base.ResetPipeline()
        if self._station_type == "Trigno Base Station":
            self.base.Configure(False, False)
        else:
            self.base.Configure()
        if not self.base.IsPipelineConfigured():
            raise RuntimeError("DelsysAPI pipeline failed to configure")

        self.emg_channels = []
        sensors = self.base.GetSensors()
        for i in range(len(sensors)):
            sensor_name = str(sensors[i].FriendlyName)
            pair_number = int(self.base.GetSensorPairNumber(i))
            for c in self.base.GetSensorChannelInfo(i):
                if c["Type"] != "EMG" or c["Enabled"] != "True":
                    continue
                self.emg_channels.append(EmgChannel(
                    sensor_index=i,
                    sensor_name=sensor_name,
                    name=str(c["Name"]),
                    guid=str(c["Guid"]),
                    sample_rate=float(c["Sample Rate"].replace(",", ".")),
                    pair_number=pair_number,
                ))
        if not self.emg_channels:
            raise RuntimeError("No enabled EMG channels found after Configure()")
        return self.emg_channels

    def start(self):
        """Start streaming; returns the monotonic time right after Start() returned."""
        with self._lock:
            self._pending = [[] for _ in self.emg_channels]
        self.base.Start(False)
        t0 = time.monotonic()
        self.streaming = True
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()
        return t0

    def stop(self):
        self.streaming = False
        if self._poll_thread is not None:
            self._poll_thread.join(timeout=2.0)
            self._poll_thread = None
        self.base.Stop()

    def drain(self):
        """Return and clear pending samples: list (per channel) of float lists."""
        with self._lock:
            out = self._pending
            self._pending = [[] for _ in self.emg_channels]
        return out

    def _poll_loop(self):
        guids = [ch.guid for ch in self.emg_channels]
        while self.streaming:
            if self.base.CheckDataQueue():
                try:
                    data = self.base.PollDataByString()
                    keys = set(str(k) for k in data.Keys)
                    with self._lock:
                        for ci, guid in enumerate(guids):
                            if guid in keys:
                                self._pending[ci].extend(float(v) for v in data[guid])
                except Exception as e:  # keep polling; surface in console
                    print(f"Delsys poll error: {e}")
            else:
                time.sleep(0.001)
