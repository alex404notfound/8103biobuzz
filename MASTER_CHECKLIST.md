# Robot setup and tuning: master checklist

Use this as the order of work. The linked guides contain the wiring, controls,
measurements and tuning instructions; the linked Java files are the OpModes to run.
Check an item only after testing it on the robot. Dashboard values must be saved
back into source to survive an app restart.

**Current state:** the turret, linked flywheel, hood and auto-aim have separate
practice OpModes. A combined competition scoring routine is still to be built.
The turret and flywheel bench steps can run before the chassis is fully tuned.

## Setup and individual mechanisms

- [ ] **1. Build, install and match the hardware configuration.** Follow
  [START_HERE](START_HERE.md) and [build/install setup](docs/TUNING.md#build-and-install).
  Check [dependency versions](docs/DEPENDENCIES.md) and the hardware names in each
  mechanism guide. Complete the full install before starting hardware tests.

- [ ] **2. Verify drive directions and Pinpoint measurements.** Follow
  [chassis and Pinpoint setup](docs/TUNING.md#1-confirm-the-chassis-and-pinpoint).
  Run [Pinpoint Check](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/testing/PinpointCheck.java)
  and the AutoTune mecanum/Pinpoint procedures linked in step 9. Confirm measured
  travel and rotation agree with the robot before trusting field position.

- [ ] **3. Configure and tune the Axon turret.** Run
  [Axon Turret Test](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/testing/AxonTurretTest.java).
  Complete [wiring, gearing, limits and zeroing](docs/AXON_TURRET_TEST.md), then
  [PID, kS/kV/kA and motion-profile tuning](docs/AXON_TURRET_TUNING.md).
  Finish with repeatable moves in both directions and verified travel limits.
  Positive turret angle must mean counterclockwise viewed from above for auto-aim.

- [ ] **4. Tune the two motors driving the linked flywheel.** Run
  [Shooter Flywheel Tuning](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/testing/ShooterFlywheelTuning.java)
  using [the flywheel guide](docs/FLYWHEEL_TUNING.md).
  Connect the `launcherLeft` encoder; it is hard-coded as the only RPM source.
  Verify both motor directions using that shared RPM reading, then tune
  feedforward and feedback. Check settling and recovery after a shot.

- [ ] **5. Calibrate hood compression and ball presets.** Run
  [Launcher Prototype](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/testing/LauncherPrototype.java)
  using [the launcher guide](docs/LAUNCHER_PROTOTYPE.md).
  Measure hood travel and separate small/large-ball settings; verify them with
  the tuned flywheel. Use its launcher controls; its chassis tag-alignment control
  assumes a fixed camera and is unsuitable for the turret-mounted Limelight.

## Vision and shooting measurements

- [ ] **6. Calibrate turret-mounted vision and field-side detection.** Run
  [Hive Auto Aim Test](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/testing/HiveAutoAimTest.java)
  using [the auto-aim guide](docs/HIVE_AUTO_AIM.md).
  Use a combined drive + Pinpoint + turret + Limelight hardware configuration.
  Measure camera/pivot/muzzle geometry and the opening offset. Verify UP/DOWN
  recognition and candidate angles before enabling aim. Apply a measured starting
  pose for field-side labels; test stale tags, tipping, travel limits and cancellation.
  This OpMode does not run the launcher.

- [ ] **7. Establish which shots actually work.** Follow
  [launcher preset calibration](docs/LAUNCHER_PROTOTYPE.md#measuring-hood-compression)
  and [shallow-angle shot guidance](docs/HIVE_AUTO_AIM.md#why-shallow-shots-can-miss-even-with-correct-horizontal-aim).
  Record ball type, distance, approach angle, RPM, hood setting and shot results.
  Start with frontal shots. Use measured results to restrict the aiming window
  and justify any lateral bias; azimuth alignment alone is not shot readiness.

## Remaining robot systems

- [ ] **8. Calibrate the Brushland sensor, if installed.** Follow
  [the color-sensor guide](docs/BRUSHLAND_COLOR_TEST.md), in its programming order:
  [Brushland Configure](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/ConfigureColorRangefinder.java),
  [Brushland Hue Check](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/testing/BrushlandHueCheck.java),
  then reprogram digital mode and run
  [Brushland Color Test](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/testing/BrushlandColorTest.java).
  Validate real balls and empty/unknown samples at the installed sensing distance.
  Color classification does not yet implement inventory counting or feeding.

- [ ] **9. Finish Pedro path tuning before autonomous.** Follow
  [Pedro 3 AutoTune](docs/TUNING.md#2-tune-pedro-3-with-autotune).
  The web procedures are registered in
  [Tuning.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Tuning.java);
  they are AutoTune procedures, not a Driver Station OpMode named "Tuning."
  Save measured settings in
  [Constants.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Constants.java),
  complete Foresight calibration and validate a short path before larger routes.

- [ ] **10. Build and validate the combined match program.** Use
  [the software requirements and remaining work](docs/BIOBUZZ_SOFTWARE_REQUIREMENTS.md#implementation-order-and-acceptance-checks).
  Integrate turret aiming, flywheel/hood readiness, intake/inventory, feeding,
  driver override and match-phase controls. Address the documented competition
  telemetry configuration before match use. Starting scaffolds are
  [Skeleton TeleOp](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/teleop/SkeletonTeleOp.java),
  [Red Skeleton](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/autos/RedSkeletonAuto.java)
  and [Blue Skeleton](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/autos/BlueSkeletonAuto.java).
  These are not completed scoring routines. Replace example start poses with
  measurements using
  [FieldConstants.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/FieldConstants.java).

## Before calling the robot ready

- [ ] Save the measured constants, hardware configuration and shot results.
- [ ] Run the [complete build and tests](docs/TUNING.md#build-and-install).
- [ ] Retest the combined controls on hardware: cancel/STOP, lost vision,
  localization loss, interrupted feeds and actual shots.
- [ ] Complete the [match acceptance checks](docs/BIOBUZZ_SOFTWARE_REQUIREMENTS.md#implementation-order-and-acceptance-checks)
  with the final mechanisms and field setup. A successful build does not check
  physical calibration or shooting accuracy.
