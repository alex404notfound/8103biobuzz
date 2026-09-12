# 8103 BIOBUZZ

Open this folder in Android Studio, or build without changing your global Java setup:

```sh
python3 tools/run_gradle.py --doctor
python3 tools/run_gradle.py
```

Robot code is in `TeamCode/src/main/java`. Installed packages include Pedro
Pathing 2.1.0-alpha.2, Ivy 1.0.0, FTCLib core 2.1.1, Sloth 0.2.4, and the matching
Dashboard/Panels variants. FTC SDK is **12.0.0**.

Read [the tuning guide](docs/TUNING.md) for hardware configurations and checks.
The first robot deployment requires `python3 tools/deploy.py --full` after joining
robot Wi-Fi. Use Driver Station 12.0. Nothing in the build commands contacts a robot.

BIOBUZZ tags move: keep Limelight absolute-pose correction disabled for game tags.
Motor/pod tuning and the example autonomous coordinates still need your robot and
field measurements. [Season repository workflow](docs/NEW_SEASON.md).
