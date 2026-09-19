# Axon turret bench test

Run **Axon Turret Test** in the **Prototyping** group. This separate OpMode needs
only the turret servo and its analog feedback; the launcher, Limelight, drivetrain
and Pinpoint are not required. It provides manual jogging, a known-position zero,
configurable gearing/travel limits, and hold-to-run profiled PID + feedforward control.
It does not yet aim the turret using AprilTags.

After setup, use [the full tuning guide](AXON_TURRET_TUNING.md) for the motion
profile, kS/kV/kA measurements, P/I/D tuning, telemetry graphs and troubleshooting.

The design follows the analog-feedback and accumulated-rotation approach in
the [RTPAxon reference you supplied](https://github.com/The-Robotics-Catalyst-Foundation/FIRST-Opensource/tree/main/FTC/RTPAxon).
This is an independent implementation using the existing FTC SDK; no additional
library is required. The reference is MIT licensed and credited to the Robotics
Catalyst Foundation. The reusable controller is `subsystems/AxonTurret.java`;
`math/ContinuousAngleTracker.java` handles encoder rollover.

## Connect and configure

1. Identify the Axon model before programming it. For MAX/MINI **MK2**, use the
   [MK2 programmer instructions](https://docs.axon-robotics.com/servos/programmer)
   and the matching model's CR firmware through **UPF**. For MAX+/MINI+/MICRO+,
   follow the [legacy programmer instructions](https://docs.axon-robotics.com/archive/programmer).
   The programmer generations are incompatible. Selecting CR Servo in the robot
   configuration does **not** change the physical servo's firmware mode.
2. Connect the normal three-pin lead to a servo port. Connect the extra feedback
   lead through a correctly wired harness to a Hub **analog signal** channel with
   common ground. The [REV analog pinout](https://docs.revrobotics.com/duo-control/sensors/analog)
   labels blue as channels 0/2 and white as 1/3. Verify the connector's pinout;
   the feedback signal does not go to a motor encoder, digital, or I2C port.
3. In the robot configuration, name a **Continuous Rotation Servo** `turret` and
   an **Analog Input** `turretEncoder`. Both names are editable in `AxonTurret`
   before INIT. `teamconfig8103_turret_bench.xml` is an example using servo port 0
   and analog channel 0 on the Control Hub; change it to match actual wiring.
4. Install the full APK for the first test so the new OpMode and XML resource are
   available. Select the matching configuration and press INIT. Voltage and
   wrapped shaft angle should appear without a movement command.

The default feedback scale is **0–3.3 V for one shaft revolution**, as used by
[SolversLib's Axon encoder support](https://docs.seattlesolvers.com/features/hardware/servos).
`encoderMinVolts` and `encoderMaxVolts` are adjustable. Do not automatically use
`AnalogInput.getMaxVoltage()` as the encoder scale: the Hub's supported input
range and the servo's output range are different quantities.

This test retains the SDK's default 600–2400 microsecond PWM range, with zero
power at its 1500 microsecond midpoint. Verify that zero power actually stops
your CR-programmed servo before testing movement. Model-specific full-range
PWM configuration can be added later if needed; this test does not flash firmware
or add neutral trim.

## Set the mechanism values

Open Dashboard and expand **AxonTurret**. Hardware names are read at INIT.
Calibration changes invalidate the position reference; changes while a motion
request is active also latch a fault, even at a stopped travel limit. Re-establish
zero after those changes.

| Setting | Meaning |
| --- | --- |
| `maximumServoDegreesPerSecond` | Required upper bound on **servo output shaft** speed at your supply voltage, including margin. Starts at 0, so movement is disabled until set. For a datasheet speed of `s` seconds/60 degrees, use `60/s` plus margin. Do not divide this bound by turret gearing. |
| `maximumSampleGapMs` | Largest permitted sample interval; default 100 ms. Gaps that could hide half a shaft turn, including a 2-degree noise allowance, are also rejected. |
| `servoTurnsPerTurretTurn` | Positive reduction ratio. Example only: 20-tooth servo gear driving a 100-tooth turret gear gives `100/20 = 5`. Starts at 0, so PID/zero are unavailable until set. |
| `encoderSign` | `+1` or `-1`: choose which measured direction is positive turret rotation. |
| `servoSign` | `+1` or `-1`: positive jog must increase signed shaft angle. Independent of `encoderSign`. |
| `directionsVerified` | Set true only after the low-power jog check confirms that relationship. |
| `limitTravel` | Default true. Keep enabled for hard stops or cables. Disable only for a mechanism physically able to rotate continuously. |
| `minDegrees`, `maxDegrees` | Measured turret travel relative to chosen zero; range must contain zero. Both start at 0 and must be configured when limits are enabled. Allow room for stopping before physical stops. |
| `manualPower`, `maximumPower` | Full-stick manual power defaults to 0.10; global output cap defaults to 0.15. Manual power cannot exceed 0.20. |
| `kP`, `kI`, `kD` | PID in turret degrees and seconds. Initial P=0.01, I=D=0 are bench starting values, not tuned gains. |
| `kS`, `kV`, `kA` | Friction, velocity and acceleration feedforward in normalized power units; all start at 0 until tuned. |
| `profileEnabled` | Default true. Generates intermediate position/velocity/acceleration targets. |
| `maxProfileVelocityDegreesPerSecond`, `maxProfileAccelerationDegreesPerSecondSquared` | Planned turret speed and acceleration, initially 30 deg/s and 60 deg/s². Tune to the mechanism. |

## First movement and zero

1. With the mechanism clear of its stops, set the speed bound and press START.
   Release RT/LB and center the left stick to arm controls.
2. Hold **LB** and move **left stick X** a little. Release LB to remove power.
   Confirm smooth encoder readings and that positive input increases **Unwrapped
   signed shaft deg**. Adjust the signs as necessary. There are **no turret travel
   limits before zero**; this raw jog is for supervised setup only. Missing encoder
   progress in the commanded direction causes a latched fault.
3. Enter the measured ratio and travel limits and set `directionsVerified=true`.
   Align the turret with the physical position you want to call zero, release
   motion controls, and let the shaft stop. Press **B**. Telemetry must show
   **Zero established: true**. No automatic homing motion occurs.
4. Choose a small target with the D-pad or Dashboard's **AxonTurretTest.targetDegrees**.
   Hold **RT** to run the profiled controller; release it to remove power. Follow
   [the tuning sequence](AXON_TURRET_TUNING.md) to measure feedforward and tune
   feedback. I starts at zero, is bounded by `integralLimit`, and has anti-windup.

| Gamepad 1 control | Action |
| --- | --- |
| LB + left stick X | Manual jog; overrides RT. Applies travel limits after zero. |
| RT | Hold profiled PID + feedforward to the chosen target. |
| D-pad left/right | Decrease/increase target by `targetStepDegrees` (default 5). |
| A | Set target to zero; motion still needs RT. |
| B, with motion released | Mark current physical position as zero and reset target. |
| Y, with motion released | Clear a latched fault. Re-establish zero before PID. |
| X | Cancel; require release of motion controls before restarting. |
| STOP | Command zero power and close the controller. |

Gearing uses **unwrapped** shaft travel, so a 359-to-1 degree encoder transition
adds 2 degrees instead of jumping backward. PID uses unwrapped turret error;
it cannot take a wrapped shortcut through a cable limit. With continuous travel
enabled, targets still represent accumulated degrees rather than shortest-path
headings.

The encoder reports only the current shaft revolution. Software cannot recover
earlier full turns after reboot or a sufficiently long sampling gap. Each run
therefore needs a new physical zero. Invalid voltage, implausible movement,
ambiguous timing, missing progress, or move timeout stops output and latches a
fault. Release the controls, diagnose it, press Y, realign and press B. An in-range
frozen/disconnected analog signal cannot always be distinguished from a stopped
shaft; the movement watchdog helps but is not a hardware limit switch.

This is a practice test with Dashboard telemetry. Software limits do not prevent
coasting or supply physical stops. PID gains, encoder wiring, neutral, gearing,
and travel must be verified on the actual mechanism before integration with
launcher/AprilTag routines.

## Validation

The complete Java suite passes: **246 tests across 32 suites**, including 60
turret/tracker/profile/OpMode tests. Debug APK and Sloth builds both pass. Coverage
includes encoder rollover, gearing, timing loss, profiling and reversal,
feedforward saturation and anti-windup, planned braking, blocked reads, fault
recovery, zeroing controls, and terminal shutdown. Hardware has not been
connected or tested.
