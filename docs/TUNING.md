# Team 8103: setup and tuning

Open **`~/Downloads/8103biobuzz`** on the prepared computer. All commands in
this guide run from that project's root. Edit robot code under
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode`.

## Build and install

```sh
python3 tools/run_gradle.py --doctor
python3 tools/run_gradle.py
```

The doctor checks local tools. The default build runs Java tests and assembles
the full APK. For an explicit complete build including Sloth:

```sh
python3 tools/run_gradle.py :TeamCode:testDebugUnitTest :TeamCode:assembleDebug :TeamCode:assembleSloth
```

After joining robot Wi-Fi, install the **full APK first**:

```sh
python3 tools/deploy.py --full
```

The helper uses the installed tools and defaults to `192.168.43.1:5555`. For a
specific robot, append its address: `python3 tools/deploy.py --full 10.8.1.2:5555`.
After ordinary Java code changes, omit `--full` to deploy Sloth. SDK, library,
plugin, manifest, and resource changes require another full install. `./push.sh`
and `./push.ps1` are convenience wrappers; their full-install options are
`--full` and `-Full`, respectively. All deployment commands target one device
and preserve build/install failures.

This BIOBUZZ project uses SDK **12.0.0**. Install the **12.0 Driver Station** for
this season. The IDE requirement is Android Studio **Narwhal 3 Feature Drop or
later**. [Official release notes](https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases/tag/v12.0).

Local tool locations are in ignored `.ftc-tools.json`. If you move the project,
update its `tools_home` or set `FTC_TOOLS_HOME`. On another computer install the
matching Java/Android tools and point the runner at a directory containing
`jdk25` (a JDK home or macOS bundle; required by Load 0.3.0), `sdk`, and writable `gradle-home` and
`android-user` folders.

## Choose the hardware configuration

Full-install the XML resources, then select the matching configuration on the
Driver Station. Verify hub identities and actual wiring before INIT.

| Configuration | Hardware | OpModes/profile |
| --- | --- | --- |
| `teamconfig8103` | Control Hub, four drive motors, `pinpoint` | Skeleton TeleOp, skeleton autos, AutoTune; `DRIVE_ONLY` |
| `teamconfig8103_vision` | Drive hardware plus `limelight` 3A | Skeleton Vision TeleOp, Limelight Check; `DRIVE_AND_VISION` |
| `teamconfig8103_example` | Vision hardware plus Expansion Hub and `example` motor | Explicit `Robot.HardwareProfile.ALL` when using the mechanism example |

Drive names are `frontLeft`, `frontRight`, `backLeft`, and `backRight`.
Hardware XML lives in `TeamCode/src/main/res/xml`. Keep names and ports consistent
with code. The default Robot does not require the optional camera or example
motor. **Pinpoint Check** maps only the Pinpoint and never commands drive motors.

## 1. Confirm the chassis and Pinpoint

**Nothing in the old template was measured on this robot.** The motor names and
directions are initial examples. Pod offsets now start at zero. Configure
`pedroPathing/Constants.java`: `drivetrainConfig` holds motor names/directions;
`localizerConfig` holds the Pinpoint name, pod preset, encoder directions and
offsets. The current preset is `goBILDA_4_BAR_POD`; choose the actual pod model.
The code uses a goBILDA **Pinpoint** computer. Pods wired directly to hub motor
encoders require a different localizer.

1. Select **Pinpoint Check**, INIT, keep the robot stationary, and press **X**.
   Wait for READY with no pending calibration.
2. START and push by hand. At heading zero, forward increases X, left increases
   Y, and counterclockwise rotation increases heading. Check counts and distance.
3. Correct the pod preset/directions and repeat. Use the AutoTune mecanum check
   with wheels raised to verify all motor directions before floor tests.

Follower construction preserves the device pose and does not reset/calibrate it.
INIT X explicitly recalibrates the gyro while retaining field pose. Robot motion
requires a recent completed READY sample with finite position and velocity.

## 2. Tune Pedro 3 with AutoTune

Full-install the APK, connect to the robot Wi-Fi, and open
**http://192.168.43.1:10158** (use the robot's address if different). This project
registers team procedures for mecanum, Pinpoint, Foresight and path checks.
The old Pedro 2 selector and predictive-braking tuners have been replaced.

Each phase starts with motors stopped. **X** recalibrates Pinpoint while
stationary; wait at least 300 ms and for READY, then release/press **A** to arm.
Manual measurement phases use A to accept the result. **B**, Driver Station STOP,
web STOP, localization faults and timeouts abort and stop the motors; recovery
does not restart a stopped phase. Confirm each phase's travel area before arming;
Foresight calibration includes full-power runs.

1. **Mecanum:** verify one motor at a time with wheels raised. Record corrected
   names/directions in `Constants.drivetrainConfig`.
2. **Pinpoint:** verify directions and measured travel; correct Constants and
   repeat if either disagrees with the physical movement. Run the offset procedure:
   rotate in place 180–240° in either direction and accept the measurement. It temporarily
   zeros offsets and restores them on exit, including faults and STOP. Save the
   measured forward-pod offset in `localizerConfig.xPodOffset` and lateral-pod
   offset in `localizerConfig.yPodOffset` (inches). Translation during rotation
   invalidates the measurement.
3. **Foresight:** run the new velocity, deceleration, braking, heading and
   translation identification steps. Copy the generated `foresightConfig` into
   Constants, including its imports. Set **`foresightTuned = true` only after
   replacing the scaffold with those measurements** and validating the hardware.
4. Full-install for library/resource changes; use Sloth for ordinary Java changes.
   Validate a short path and STOP before larger paths or competition autonomous.

The scaffold uses zero feedback/braking gains and dummy positive scale values
solely so TeleOp and sensor checks can initialize. It is not a robot tune.
Autonomous refuses START while `foresightTuned` is false. Optional TeleOp heading
hold gains (`Drivetrain.headingP/I/D`) also start at zero and need tuning.

Dashboard/Panels remain available for live telemetry and mechanism PID tuning.
Record final values in source so app restarts retain them.
[Pedro 3 AutoTune documentation](https://pedropathing.com/docs/pathing/tuning).

## 3. Limelight in BIOBUZZ

**Keep `enableRelocalization = false` for BIOBUZZ competition tags.** FIRST states
that these tags move and cannot provide absolute field localization. Pinpoint
provides the field pose; Limelight can still provide target-relative information
such as tx/ty. The existing stationary pose-correction code is retained for a
separate practice setup with fixed, surveyed tags. Do not enable it using the
BIOBUZZ game tag map. [SDK 12 release notes](https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/v12.0/README.md#version-120-20260907-090034).

The following frame-calibration steps apply only to fixed practice tags:

Select the vision hardware configuration. In the Limelight web UI, configure an
AprilTag pipeline with Full 3D enabled, the correct field map, and measured camera
position/orientation relative to the robot. The FTC driver and MegaTag2 support
are already in the SDK. [Limelight FTC guide](https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming).

Run **Limelight Check** while stationary. It has no driving controls. In INIT,
**X** recalibrates Pinpoint. The `Limelight` Dashboard config defaults to pipeline
0; changing `pipelineIndex` requires STOP and a new INIT. Confirm the requested
and returned pipeline match, tags are detected, and heading writes/results are
fresh.

Set these field-frame values using the actual map and your Pedro coordinate convention:

| Limelight setting | Meaning |
| --- | --- |
| `frameOriginXInches`, `frameOriginYInches` | Pedro coordinates of the Limelight map's origin |
| `frameRotationDegrees` | Counterclockwise rotation from map axes into Pedro axes |
| `enableRelocalization` | Explicit opt-in after checking the transform; defaults to false |

The conversion is `Pedro XY = origin + rotate(map XY converted from meters to
inches)`. The inverse rotation converts Pinpoint heading to the yaw sent to
MegaTag2. Verify several known positions and headings, including both sides of
the field. The zero origin/rotation defaults are **uncalibrated values**, not a
claim about the next game's map. Physical vision poses are not mirrored by alliance.

Only then enable relocalization and press **Circle** after START to request one
translation correction. It keeps the Pinpoint heading and stops any active path
or end hold. Default limits require speed at most 1 in/s, turn rate at most
10°/s continuously for at least `maxStalenessMs` (100 ms by default), a correction
at most 12 inches, and data/heading age at most 100 ms. A fault or long sampling
gap restarts the stationary wait. Missing
MegaTag2 data, wrong pipelines, frozen/stale frames, invalid values, zero tags,
implausible height/tilt, or unhealthy Pinpoint data reject the correction. Inspect
the rejection telemetry before changing limits. This is a stationary correction
command; it does not continuously fuse camera measurements into Pedro.

## 4. Prepare autonomous for the actual game

Replace `robot/FieldConstants.java` and the start/path geometry in
`opmodes/autos/SkeletonAuto.java`. Their current coordinates and 141.5-inch field
width are DECODE placeholders. Confirm the new field's coordinate convention
and alliance symmetry before retaining `PoseMirror`'s X reflection.

Author shared auto geometry once, then use `transformed()` and
`transformedHeading()` for the red/blue pair. Build actions in `buildSequence()`;
put setup in `onAutoInit()`. The base refuses START until Foresight is configured and localization is healthy
and cancels an active sequence after localization faults. Validate one short
path, interruption/STOP, pose handoff to teleop, then both alliance versions on
the actual field. These software checks do not establish collision-free paths.

## FTCLib and Ivy

Separate **FTCLib core 2.1.1** is included in this project, with a host test using
the real PID controller API:

```java
import com.arcrobotics.ftclib.controller.PIDController;
```

Use its controllers and math utilities as needed. Keep **Ivy** as the only
command scheduler, and keep persistent mechanism control in each subsystem's
`periodic()`. FTCLib is a Java library installed through Gradle; it is not a
separate desktop tuning application. Dashboard/Panels provide the live tuning UI.
[FTCLib project](https://github.com/FTCLib/FTCLib).


A separate checkout of the official FTCLib **v2.1.1** source is installed on this
computer at `~/Downloads/8103Template/.tools/ftclib`. It is for examples/reference;
the robot uses the published Gradle dependency. FTCLib is a library, not a desktop
application. Ivy core + Pedro integration **1.1.0** is the sole command framework;
NextFTC is not installed.
