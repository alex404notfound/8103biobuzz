# Two-motor launcher prototype

Run **Launcher Prototype** in the **Prototyping** group. It controls two goBILDA
6000 RPM motors linked to the same flywheel, a positional hood-compression servo,
and a Limelight 3A. Both motors receive one shared control effort.
Bench mode is the default and needs no drivetrain hardware or Pinpoint.
This is a practice OpMode with Dashboard tuning, not the competition program.

For your **turret-mounted** camera, use the separate
[Hive Auto Aim Test](HIVE_AUTO_AIM.md) for CELL orientation and turret aiming.
The chassis alignment test below assumes a fixed camera aligned with the chassis;
do not use that control while the camera is rotating on the turret.

## Hardware and first installation

| Device | Default robot configuration name |
| --- | --- |
| First launcher motor, with encoder connected | `launcherLeft` |
| Second launcher motor, with encoder connected | `launcherRight` |
| Positional servo driving the rack and pinion | `hood` |
| Limelight 3A | `limelight` |

Motor names and directions are editable in the `ShooterFlywheel` Dashboard
section before INIT; the hood name is in `Launcher`.
This expects a positional servo. A continuous-rotation servo would require
position feedback and different hood control code.

Select the custom **goBILDA 6000 RPM (1:1)** motor type. It uses 28 encoder counts
per motor revolution, matching the manufacturer's
[5203-2402-0001 specification](https://www.gobilda.com/content/user_manuals/5203-2402-0001_spec_sheet.pdf).
Confirm the motor label. Any external gearing changes wheel RPM, not the motor
RPM shown here. The generic SDK goBILDA type describes a different gear ratio.

Two example configurations are included. Verify actual wiring before selection:

- `teamconfig8103_launcher_bench`: launcher on Control Hub motor ports 0/1,
  hood on servo port 0. Do not use this mapping if those ports power drive motors.
- `teamconfig8103_launcher_drive`: four-motor drive and Pinpoint on the Control
  Hub; launcher on Expansion Hub motor ports 0/1 and servo port 0. Verify the hub
  address as well as ports.

Neither configuration is selected automatically. The new motor type and XML
resources require a full APK install:

```sh
python3 tools/run_gradle.py :TeamCode:testDebugUnitTest :TeamCode:assembleDebug :TeamCode:assembleSloth
python3 tools/deploy.py --full
```

Builds do not connect to the robot. Deploy separately using the usual device
selection in [TUNING.md](TUNING.md).

## Motor setup

1. Keep `LauncherPrototype.enableDrive = false` for bench work. Open Dashboard
   on robot Wi-Fi, select **Launcher Prototype**, then INIT and START.
2. With an empty launcher, hold **GP1 LB + X** to power only the first motor at
   `ShooterFlywheel.testVoltage`; **LB + Y** powers only the second. Release to
   stop. The unpowered motor coasts and can turn through the mechanical linkage.
   Check that each powered motor drives the shared flywheel in the same shooting
   direction and that both encoders report positive RPM. Zero RPM while a motor
   turns means its encoder or configuration needs fixing before speed mode.
3. Set `ShooterFlywheel.leftReversed` / `rightReversed` for the mechanism. STOP
   and re-INIT after changing these: directions are read at INIT. Mirrored motor
   mounting may require opposite direction settings; the code does not assume it.
4. Once directions and encoder readings are verified, set
   `ShooterFlywheel.directionsVerified = true`. Hold **RT** for both motors in RPM mode.
   Initial 1000 RPM presets are bench starting values, not shooting settings.
5. Tune `smallBallRpm` and `largeBallRpm`, or use up/down for 100 RPM steps.
   The shared flywheel follows the selected setpoint. Configure and tune its
   voltage feedforward and velocity feedback in **Flywheel Tuning**, following
   [FLYWHEEL_TUNING.md](FLYWHEEL_TUNING.md). The launcher uses that same
   `ShooterFlywheel` controller and its Dashboard settings.

The motor controller now runs software velocity PID plus `kS`, `kV`, and `kA`
feedforward. Its output is volts, divided by measured battery voltage to obtain
motor power. `kV` uses volts per motor RPM; `kA` uses volts per motor RPM/second.
`maxAccelerationRpmPerSecond` ramps the reference speed. Both linked motors get
the same power, with feedback based on their average RPM and a separate encoder
disagreement check. The old REV `Launcher.velocityP/I/D/F` settings were removed:
they do not apply to this controller. Hardware stays in `RUN_WITHOUT_ENCODER` so
the Hub does not run a second velocity controller; encoder readout still works.

Both encoders must remain within tolerance of the final requested RPM, and the
ramp must finish, before `at speed` becomes true. A
speed timeout stops both motors and latches while RT is held. Release, diagnose,
then hold again to retry. Invalid or above-6000 RPM targets are rejected.
Individual tests use a limited voltage; the linked wheel still needs time to coast.

## Measuring hood compression

`hoodMin`, `hoodMax`, `smallBallHood`, `largeBallHood`, and `calibrationHood`
start at **-1**, meaning unmeasured. INIT and START do not send a servo position.

Establish the usable servo range with the rack disconnected or arranged so its
first commanded position cannot hit an end stop. Set measured `hoodMin` and
`hoodMax` in normalized servo units (0-1), then set `calibrationHood` inside that
interval. After START, press **Y without LB** to apply it. With stopped motors,
left/right jog by 0.005 within those limits. Record the working positions in
`smallBallHood` and `largeBallHood`; their numeric ordering is not assumed.

**A** selects small pollen and applies its hood preset; **B** selects big nectar
and applies its preset. Wait for the wheels to coast down before moving the
hood. Re-press A/B if a command was rejected while spinning. Dashboard preset
edits require another explicit hood command to move the servo. The reported
position is the command, not measured rack travel; readiness uses a settling
delay. Save successful values into `Launcher.java`, since runtime tuning edits
are not a durable calibration file.

## Limelight setup and aiming

Set `Limelight.pipelineIndex` before INIT. Configure an AprilTag pipeline with
**36h11** tags and **3.25-inch / 0.08255-meter** tag size. Enable individual
fiducial results, and enable Full 3D for optional range readout. See the official
[FTC Limelight guide](https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming).

| Cell | IDs |
| --- | --- |
| Red, opposite audience | 30-33 |
| Red, audience side | 34-37 |
| Blue, audience side | 38-41 |
| Blue, opposite audience | 42-45 |

Source: section 9.9 of the [BIOBUZZ manual](https://ftc-resources.firstinspires.org/ftc/game/manual).
In INIT, **GP2 LB/RB selects red/blue**; alliance locks at START. GP2 left/right
cycles tag IDs. Choose a visible tag on the upward cell and change it after a
hive tip. `targetTagId` is also editable in Dashboard. No neighboring or opposing
tag is silently substituted if your selection disappears.

Telemetry shows tag angles, turn output, and optional **camera-to-tag slant
range**. This is not horizontal launcher-to-opening distance and does not set
RPM. Unavailable 3D range displays NaN rather than a false zero-inch measurement.

For chassis aiming, set `LauncherPrototype.enableDrive = true` **before INIT**
and use a matching drive configuration. Verify drive directions and Pinpoint
using [TUNING.md](TUNING.md). GP2 sticks drive; holding **LT** turns in place
toward the selected tag. Manual turn-stick input overrides aiming. Translation
pauses while aiming; missing/stale/wrong-pipeline/wrong-alliance observations
command zero turn. A Pinpoint fault also stops the drive.

Start with low `HiveTagAim.maximumTurnPower`. Verify that turning reduces error;
reverse `turnSign` if needed. Tune `turnKp` and `targetTxDegrees` for the selected
tag and shooting station. A tag is offset from the cell opening: centering it
alone is not a calibrated trajectory. Changing tag, distance, or camera mounting
may require another offset.

`TAG ALIGNED` needs multiple fresh frames, horizontal tolerance, and a vertical
stability window. It **does not identify an upward cell or confirm a completed
tip**. The driver verifies the cell and controls manual test feeding. Moving
game tags never change absolute field pose.

## Controls after START

| Input | Action |
| --- | --- |
| GP1 RT, held | Both motors at selected RPM after direction verification |
| GP1 LB+X / LB+Y, held | Left / right motor at low test voltage; other motor coasts |
| GP1 X without LB | Cancel motor and drive output; release controls before resuming |
| GP1 A / B | Small / big ball selection and explicit hood preset |
| GP1 up / down | Raise / lower the selected RPM preset |
| GP1 Y without LB | Apply `calibrationHood` |
| GP1 left / right | Jog hood within measured limits, with motors stopped |
| GP2 left / right | Choose a tag on your alliance's hive |
| GP2 sticks | Manual drive, when enabled |
| GP2 LT, held | Turn-only tag alignment; manual turn overrides |
| Driver Station STOP | End prototype; clean up motors, drive, and camera |

No intake or feeder is assumed. Releasing RT stops motor power and the wheels
coast. During the OpMode the hood holds its last target; STOP does not command
a new position. The reusable `Launcher` and `HiveTagAim` classes are not wired
into the skeleton competition OpModes. Automatic cell-state detection, shot
calibration, feeding, and competition service configuration remain separate work.

## Validation

The launcher tests cover shared-controller delegation, live RPM presets, hood
limits and coast-down interlocks, readiness, and terminal cleanup. Flywheel
controller and tuning OpMode tests cover motor output and tuning controls;
see [FLYWHEEL_TUNING.md](FLYWHEEL_TUNING.md). Motor/servo behavior, camera mounting,
aim direction, and actual shooting presets still require bench/field testing.
