# emg_acquisition

GUI to record Delsys Trigno EMG and exo joint encoders (one arm) into CSV.

- EMG: native sensor mode closest to 2 kHz, auto-selected after scan
  (Avanti modes are fixed rates — e.g. 1259.259 / 2148.148 Hz — so exactly
  2000 Hz is generally not available; the actual rate is written to the CSV header).
- Encoders: 500 Hz from `/{side}_joint_state_broadcaster/joint_states`,
  published by the read-only exo stack that the GUI launches itself
  (`exo.launch.py control_mode:=read_only` — motors idle).

## Setup (once)

```bash
./setup_venv.sh
```

Requires: ROS 2 Jazzy + built `exo_right_arm_ws`, the Delsys
Example-Applications folder (path in `config.py`, overridable with
`DELSYS_PYTHON_DIR`), .NET runtime (already working for the vendor demo),
user in the `dialout` group.

## Run

```bash
./run.sh
```

Order matters: the Delsys connect probes every serial port on the machine,
which disturbs the Teensy links if the exo stack is already running (encoders
freeze, RF discovery can fail). Always do Delsys first:

1. **Connect**, then **Scan sensors** (sensors on, paired). The EMG mode
   nearest 2 kHz is selected automatically; override via the Mode dropdown.
2. **Launch stack** (pick arm side first) — wait until Encoders shows `alive`.
3. Set file name and optional duration (0 = manual stop), hit **Start**.
4. **Stop** (or auto-stop after the set duration).

If the encoders ever freeze after a Delsys connect attempt, unplug/replug the
Teensy USB (or power-cycle the exo) and restart the stack.

## Output

Two CSVs in `recordings/`, sharing the same time origin t0 (the instant the
EMG stream started; software sync, ~10 ms accuracy):

- `<name>_emg.csv` — `t, <sensor>|EMG...` one row per EMG sample, `t = k/fs`.
- `<name>_joints.csv` — `t, pos_*, vel_*, eff_*` one row per joint_states
  message (500 Hz), `t` stamped on arrival.

Both files start with a `#` metadata line (start time, arm, mode, actual rates).
