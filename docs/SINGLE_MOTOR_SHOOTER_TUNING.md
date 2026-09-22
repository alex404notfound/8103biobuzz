# Single motor shooter tuning

Select **Single Motor Shooter Tuning** in the **Prototyping** group. It needs
one encoder-equipped flywheel motor and one positional servo. All controls are
in FTC Dashboard under **Config > SingleMotorShooterTuning**; no gamepad is needed.

## Run it

1. Configure a motor named `launcherLeft` and a servo named `hood`, or change
   `motorName` and `servoName` in Dashboard before INIT. Plug in the motor encoder.
2. Set `ticksPerRevolution` for your motor. The default **28** matches the existing
   1:1, 6000 RPM shooter motor. RPM is motor-shaft RPM; external gearing changes
   the flywheel's RPM relative to this value.
3. Open [FTC Dashboard](http://192.168.43.1:8080/dash) while connected to the Control
   Hub network. Select the OpMode and INIT, then START. The motor stays off until
   you set `runFlywheel = true` after START. INIT and START clear old run commands.
4. Set `servoPosition` to the desired **0.0–1.0** position and save. The servo
   follows this setting after START, including while the flywheel is off.
   Values outside that range are clamped to the nearest endpoint. Choose positions
   within your mechanism's actual travel. The initial position is **0.5**.
5. Set `targetRpm`, save, then set `runFlywheel = true`. RPM and gains update live.
   Set `runFlywheel = false`, `targetRpm = 0`, or press STOP to remove motor power
   immediately. The flywheel coasts; STOP does not command a new servo position.

Hardware names and `motorReversed` apply at INIT. STOP and INIT again after editing
them. A direction edit during a run stops the flywheel until re-initialization.
Invalid control settings or encoder readings clear the run toggle and report a
status message. A control-loop gap over 250 ms also clears the run toggle.

## Tune RPM

Graph `shooter.targetRpm`, `shooter.referenceRpm`, `shooter.measuredRpm`, and
`shooter.motorPower`. The reference ramps toward the requested target so that
acceleration feedforward has a defined input.

| Dashboard field | Meaning |
| --- | --- |
| `kP`, `kI`, `kD` | PID correction from RPM error; derivative uses measured speed. |
| `kV` | Motor power per RPM; default `1 / 6000` is only a starting estimate. |
| `kA` | Motor power per RPM/second of reference acceleration; initially zero. |
| `maxAccelerationRpmPerSecond` | Reference ramp rate; initially 3000, must be positive. |
| `maxPower` | Power ceiling from 0 to 1; initially 0.5. |
| `integralLimit` | Maximum integral magnitude in RPM-seconds; initially 1000. |

Start with `kP`, `kI`, `kD`, and `kA` at zero and adjust `kV` until steady RPM is
close to the target. Add `kP` for correction and recovery. Use `kI` only if a steady
error remains, and `kD` only if needed to reduce oscillation. Tune `kA` against the
reference during spin-up; it contributes zero once reference RPM is constant.
If power stays at `maxPower`, the output is capped; more gain cannot raise it.

The loop uses `getVelocity()` to read encoder ticks/second, converts to RPM, then
applies software PID + `kV * referenceRpm + kA * referenceAcceleration` through
`setPower()` in `RUN_WITHOUT_ENCODER`. Output is limited to `0..maxPower`, with
integral anti-windup. These gains use **power**, RPM, and seconds, and are separate
from the two-motor tuner's voltage-based gains. Battery voltage is not compensated.

Dashboard values are live tuning values; copy your final settings into the code
to keep them across app restarts. See the official
[Dashboard config and graphing guide](https://acmerobotics.github.io/ftc-dashboard/features)
and [FTC motor velocity API](https://javadoc.io/static/org.firstinspires.ftc/RobotCore/10.2.0/com/qualcomm/robotcore/hardware/DcMotorEx.html).
