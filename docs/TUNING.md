# Team 8103: setup and tuning

Open **`~/Downloads/8103biobuzz`** on the prepared computer. All commands in
this guide run from that project's root. Edit robot code under
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode`. In the reusable template,
the equivalent path starts with `src/main/java` instead.

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
`jdk21` (a JDK home or macOS bundle), `sdk`, and writable `gradle-home` and
`android-user` folders. The legacy Dairy template also needs `jdk8`.

## Choose the hardware configuration

Full-install the XML resources, then select the matching configuration on the
Driver Station. Verify hub identities and actual wiring before INIT.

| Configuration | Hardware | OpModes/profile |
| --- | --- | --- |
| `teamconfig8103` | Control Hub, four drive motors, `pinpoint` | Skeleton TeleOp, skeleton autos, Tuning; `DRIVE_ONLY` |
| `teamconfig8103_vision` | Drive hardware plus `limelight` 3A | Skeleton Vision TeleOp, Limelight Check; `DRIVE_AND_VISION` |
| `teamconfig8103_example` | Vision hardware plus Expansion Hub and `example` motor | Explicit `Robot.HardwareProfile.ALL` when using the mechanism example |

Drive names are `frontLeft`, `frontRight`, `backLeft`, and `backRight`.
Hardware XML lives in `TeamCode/src/main/res/xml`. Keep names and ports consistent
with code. The default Robot does not require the optional camera or example
motor. **Pinpoint Check** maps only the Pinpoint and never commands drive motors.

## 1. Confirm the chassis and Pinpoint

The numbers in `pedroPathing/Constants.java` came from another robot. Verify motor
directions, maximum power, pod model, encoder directions, offsets, and velocities
before treating any of them as tuned. The current pod preset is
`goBILDA_4_BAR_POD`; select the preset matching your actual pods or provide the
measured custom resolution. The code expects a goBILDA **Pinpoint** computer;
pods wired directly into hub motor encoders need a different Pedro localizer.

1. Select **Pinpoint Check**, press INIT, keep the robot stationary, and press
   gamepad1 **X**. Wait for READY and no pending calibration.
2. Press START and push the robot by hand. At heading zero, forward should
   increase X, left should increase Y, and counterclockwise rotation should
   increase heading. Check both encoder counts and a measured distance.
3. Correct the pod model/directions in Constants and repeat. Check drive motor
   directions separately with wheels clear before driving on the field.

Pedro's follower construction performs its own Pinpoint pose/IMU reset during
INIT. Keep the chassis still throughout initialization. The explicit INIT X
control recalibrates the gyro without resetting the current field pose.
Readiness requires completed calibration, READY device status, and finite data.

## 2. Tune Pedro

Select **Tuning** and choose a child tuner. Keep still in INIT; **X** recalibrates
Pinpoint. Wait for READY before START. Running localization faults or **B** stop
the selected tuner and latch the abort. Press Driver Station STOP before trying
again; recovery does not restart motion. Automatic tuners command motors,
including full power, so clear the complete travel area shown by that tuner.

1. **Localization → Localization Test:** check pose scale and orientation.
2. **Localization → Offsets Tuner:** START, then rotate in place exactly 180° in
   either direction. Wait for **Calibration captured**. Record the absolute
   `strafePodX` and `forwardPodY` results, STOP, then put them in
   `Constants.localizerConstants`. The tuner temporarily zeros offsets and
   restores them when stopped. Translation during the turn invalidates the result.
3. **Automatic → Forward Velocity Tuner / Lateral Velocity Tuner:** measure this
   chassis and save `xVelocity` / `yVelocity` in `Constants.driveConstants`.
4. Use the zero-power acceleration tuners and **Predictive Braking Tuner** for
   the selected braking approach. Save the reported coefficients in Constants.
   The template supplies predictive braking coefficients that need replacing.
5. Tune **Manual → Heading Tuner**, then validate braking and path tracking.
   Translational/Drive PID tuners apply when using the PIDF drive approach.
   Finish with short **Tests → Line** runs before larger shapes.

Use Dashboard/Panels telemetry while tuning; record final values in source and
redeploy so a fresh app start retains them. Pedro's current documentation may
show APIs newer than this project's pinned alpha; match edits to the supplied
Constants/Tuning classes. See the official
[automatic tuning guide](https://pedropathing.com/docs/pathing/tuning/automatic)
and [predictive braking guide](https://pedropathing.com/docs/pathing/tuning/drive-algorithm/predictive/about).

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
put setup in `onAutoInit()`. The base refuses START without healthy localization
and cancels an active sequence after localization faults. Validate one short
path, interruption/STOP, pose handoff to teleop, then both alliance versions on
the actual field. These software checks do not establish collision-free paths.

## FTCLib and Ivy

Separate **FTCLib core 2.1.1** is included in both builds, with a host test using
the real PID controller API:

```java
import com.arcrobotics.ftclib.controller.PIDController;
```

Use its controllers and math utilities as needed. Keep **Ivy** as the only
command scheduler, and keep persistent mechanism control in each subsystem's
`periodic()`. FTCLib is a Java library installed through Gradle; it is not a
separate desktop tuning application. Dashboard/Panels provide the live tuning UI.
[FTCLib project](https://github.com/FTCLib/FTCLib).
