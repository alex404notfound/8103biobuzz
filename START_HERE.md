# 8103 BIOBUZZ

Follow the [master setup and tuning checklist](MASTER_CHECKLIST.md) for the order
of work and links to each guide and test OpMode.

This is the season project: **FTC SDK 12.0.0, Pedro 3.0.1, Ivy 1.1.1 and
AutoTune 1.0.1**. FTCLib core 2.1.1, Sloth 0.3.2, Dashboard and Panels are included.
See [all dependency versions](docs/DEPENDENCIES.md).

Normal robot telemetry goes to Driver Station and FTC Dashboard. Panels remains
available for Pedro AutoTune; normal OpModes do not send it a duplicate telemetry
stream. This does not disable the libraries' background services for matches.

Read [the BIOBUZZ software requirements](docs/BIOBUZZ_SOFTWARE_REQUIREMENTS.md)
for the manual review and implementation order. This is still a driving and
tuning foundation: competition scoring routines, match-phase controls, and a competition
configuration that disables third-party telemetry services are not implemented.

For the geared Axon turret, start with the standalone
[Axon Turret Test](docs/AXON_TURRET_TEST.md): CR servo + analog encoder, manual
jogging, configurable gearing/limits, and profiled PID + feedforward position control.
Then follow [the turret tuning guide](docs/AXON_TURRET_TUNING.md) for kS/kV/kA,
P/I/D, profile limits and Dashboard graphs.

For shooter tuning, use [Shooter Tuning](docs/SHOOTER_TUNING.md): one TeleOp
for two linked flywheel motors and one positional servo. RPM, PID/feedforward,
and servo position are controlled from one Dashboard section. Both motors brake
at zero output. No camera or drivetrain is required.

For the turret-mounted Limelight, use [Hive Auto Aim Test](docs/HIVE_AUTO_AIM.md)
to calibrate tag-based CELL orientation and turret azimuth, compensate camera
delay with Pinpoint/turret history, and display the robot's estimated field side.
It requires measured camera/opening geometry and does not run the launcher.

Install Python 3, **JDK 21**, and the Android SDK with
`platforms;android-30`, `platform-tools`, and `build-tools;35.0.0`.
The build runner expects the JDK under `.tools/jdk21` and the SDK under
`.tools/sdk` in this checkout. It also accepts a macOS JDK's `Contents/Home`
layout or an extracted vendor directory inside `jdk21`. An existing JDK 25 under
`.tools/jdk25` also works; the runner prefers JDK 21 when both are present.

To share tools between checkouts, set `FTC_TOOLS_HOME` to an absolute tools
directory, or create an ignored `.ftc-tools.json` in this folder with a
`tools_home` property containing that absolute path. The directory must contain
the same `jdk21` (or `jdk25`) and `sdk` layout. The environment variable takes
precedence over the config file; otherwise the runner uses this checkout's
`.tools` directory.
These tools and local settings are not included in Git.

Build from this folder (use `python` or `py -3` on Windows if that is how Python 3
is installed):

```sh
python3 tools/run_gradle.py --doctor
python3 tools/run_gradle.py
```

For Android Studio, open this checkout and set **Settings → Build, Execution,
Deployment → Build Tools → Gradle → Gradle JDK** to the installed JDK 21 home
(or an existing JDK 25). Set **Settings → Languages & Frameworks → Android SDK**
to the installed SDK directory. Android Studio stores project settings in ignored
`.idea/` files and the SDK path in ignored `local.properties`; configure them on
each computer.
The Python runner selects its tools separately using the locations above.

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

For the Brushland Labs Color Rangefinder, follow the
[BIOBUZZ color calibration and digital bulk-read test guide](docs/BRUSHLAND_COLOR_TEST.md).
