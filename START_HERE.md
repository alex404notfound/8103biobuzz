# 8103 BIOBUZZ

This is the season project: **FTC SDK 12.0.0, Pedro 3.0.0, Ivy 1.1.0 and
AutoTune 1.0.0**. FTCLib core 2.1.1, Sloth 0.3.0, Dashboard and Panels are included.
See [all dependency versions](docs/DEPENDENCIES.md).

Build from this folder:

```sh
python3 tools/run_gradle.py --doctor
python3 tools/run_gradle.py
```

The runner selects the installed **JDK 25** required by Load 0.3.0. This folder's
local Android Studio settings also select that JDK and the installed Android SDK.
On another computer, set **Settings → Build, Execution, Deployment → Build Tools
→ Gradle → Gradle JDK** to JDK 25. On this computer its home is
`~/Downloads/8103Template/.tools/jdk25/Contents/Home`.

**All robot constants are setup examples, not measurements.** Confirm wiring,
motor directions and pod type, then follow [the tuning guide](docs/TUNING.md).
Autonomous paths remain disabled until the Foresight measurements are entered
and `Constants.foresightTuned` is set to `true`. TeleOp and calibration work before
that. AutoTune is at `http://192.168.43.1:10158` while connected to robot Wi-Fi.

The first deployment after this library upgrade requires a **full install**:

```sh
python3 tools/deploy.py --full
```

Use Driver Station 12.0. Build commands do not contact a robot.
Robot code is under `TeamCode/src/main/java`. BIOBUZZ tags move, so Limelight
absolute-pose correction stays disabled for game tags.
[Project locations](docs/NEW_SEASON.md).
