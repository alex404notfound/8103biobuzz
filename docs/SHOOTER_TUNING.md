# Shooter tuning

Select **Shooter Tuning** in the **Prototyping** group. It uses exactly two
motors driving one linked flywheel and one positional servo. All controls are
in FTC Dashboard under **Config > ShooterTuning**; no gamepad is needed.

## Hardware and startup

| Hardware | Default configuration name |
| --- | --- |
| Left flywheel motor, with encoder | `launcherLeft` |
| Right flywheel motor | `launcherRight` |
| Hood / compression servo | `hood` |

The left encoder is the only RPM source. Both motors receive the same output
from one RPM controller. Set `leftMotorName`, `rightMotorName`, `servoName`,
`leftReversed`, and `rightReversed` before INIT. STOP and INIT again after
changing names or directions. Set `ticksPerRevolution` for the feedback motor
(default 28); reported RPM is at the motor shaft.

1. Open [FTC Dashboard](http://192.168.43.1:8080/dash) on the robot network.
2. Select **Shooter Tuning**, INIT, then START. Both motors stay off until a
   fresh `runFlywheel = true` command after START.
3. Set `targetRpm`, then set `runFlywheel = true`. Targets and gains update live.
4. Set `targetRpm = 0`, `runFlywheel = false`, or press STOP to immediately
   command zero power to both motors. Both motors use `BRAKE` at zero output.

## Controls

| Dashboard field | Action |
| --- | --- |
| `runFlywheel` | Master motor run toggle; cleared at INIT, START, STOP, or a fault. |
| `targetRpm` | Motor-shaft target, 0 to 6000 RPM. |
| `maxPower` | Shared output ceiling; defaults to 1.0 (100%). |
| `servoPosition` | Live positional-servo command, clamped to 0 to 1. |

There is one RPM control path. Invalid settings or encoder readings, a direction
edit, or a loop gap over 250 ms clear the run toggle and show a status message.

## Tune RPM

Graph `shooter.targetRpm`, `shooter.referenceRpm`, `shooter.measuredRpm`,
`shooter.feedforwardPower`, `shooter.pPower`, `shooter.iPower`, `shooter.dPower`,
`shooter.requestedPower`, and `shooter.leftPower`. `shooter.rightPower` is always
the same applied power. `shooter.outputLimited` shows when the controller's
request is clipped to the output range.

The reference ramps upward; a lower target takes effect immediately. Only the
left encoder feeds the controller. Both motors use `RUN_WITHOUT_ENCODER` with
software velocity control, so there is no second Hub velocity controller.

| Field | Units / initial value |
| --- | --- |
| `kP`, `kI`, `kD` | PID gains using power, RPM, and seconds; initially zero. |
| `kS` | Constant feedforward power while reference RPM is positive; initially zero. |
| `kV` | Power per RPM; preserves the current tuning value `0.00069`. |
| `kA` | Power per RPM/second of reference acceleration; initially zero. |
| `maxPower` | Power ceiling for each motor, 0 to 1; initially 1.0. |
| `maxAccelerationRpmPerSecond` | Positive upward reference ramp rate; initially 3000. Lower targets apply immediately. |
| `integralLimit` | Integral magnitude limit in RPM-seconds; initially 1000. |

The controller adds feedforward and feedback, then clips the sum once:

```text
error = referenceRPM - measuredRPM
feedforward = kS + kV * referenceRPM + kA * referenceAcceleration
feedback = kP * error + kI * accumulatedError - kD * measuredAcceleration
motorPower = clamp(feedforward + feedback, 0, maxPower)
```

`kS` applies only at a positive reference; a zero target bypasses the controller
and commands zero power. P adds power below the reference and subtracts it
above the reference. At the reference, P is zero and feedforward supplies the
base output. P does not modify `kV`; it corrects the speed error left by the
feedforward estimate. This is the standard
[additive PID/feedforward arrangement](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/combining-feedforward-feedback.html).

Start with `kP = kI = kD = kA = 0`. Adjust `kV` until steady measured RPM is
close to the target, then add `kP` for correction and recovery. Tune at several
speeds below output saturation. Add `kI` only for persistent steady error and
`kD` only if needed to reduce oscillation. Tune `kA` during reference acceleration;
it contributes zero at constant reference speed. Target or gain changes clear
accumulated I error; integral anti-windup limits accumulation at saturated output.

The current `kV = 0.00069` requests full power at about **1449 RPM**, even with
all other gains zero. At 2000 RPM its feedforward alone is **1.38**, which clips
to 1.0. P can therefore appear ineffective while clipped, or subtract power if
measured speed is too high. Use the separate term graphs to distinguish this
from an incorrect PID calculation; no gain values have been calibrated here.

Any zero output uses `BRAKE`, including a negative controller request clipped
to zero during deceleration. The controller never commands reverse. Braking
reduces coasting; actual RPM response still needs to be checked on the mechanism.

These gains produce **motor power**, not volts; battery voltage is not
compensated. They are local to `ShooterTuning`, separate from the existing
voltage-based `ShooterFlywheel` subsystem used by the reusable `Launcher` class.
Save measured values into source to keep them across app restarts.

## Measure hood compression

`servoPosition` applies on the first loop after START, even while the motors are
off; its default is 0.5. INIT does not move the servo. Choose a starting position
within the mechanism's travel, and measure usable endpoints before moving an
attached rack. STOP leaves the last commanded position unchanged.

With the flywheel stopped, adjust `servoPosition` in small steps and record
useful positions for each ball type. Then test shots and record ball type,
distance, RPM, servo position, and results. The reported servo position is a
command, not measured rack travel. This tuner has no camera, drivetrain,
automatic aiming, or feeder dependency.
