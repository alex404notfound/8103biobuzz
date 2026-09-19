# Tuning the Axon turret: motion profile, PID, kS, kV and kA

Use **Axon Turret Test** in the **Prototyping** group. First complete the wiring,
CR firmware, encoder direction, gearing, travel-limit and zero checks in
[the bench setup guide](AXON_TURRET_TEST.md). All tuning below uses that same
standalone test; no launcher or camera is required.

The recommended order is **mechanical setup → gentle profile → kS/kV estimates →
P and D → kA → final speed/acceleration → I only if needed**. Change one setting
at a time. These values must be measured on the assembled turret; the source
defaults are starting values, not a finished tune.

## Open FTC Dashboard and tune live

1. Connect a laptop to the robot's Wi-Fi and open
   [FTC Dashboard on the Control Hub](http://192.168.43.1:8080/dash).
   A phone Robot Controller uses `http://192.168.49.1:8080/dash` instead; these are
   the addresses in the [official Dashboard setup guide](https://acmerobotics.github.io/ftc-dashboard/gettingstarted).
2. On the Driver Station, select **Axon Turret Test** under **Prototyping**, then
   INIT. Use the physical Driver Station gamepad for the hold-to-run controls.
3. In Dashboard's configuration pane, expand **AxonTurret** for `kP`, `kI`, `kD`,
   `kS`, `kV`, `kA`, the profile speed/acceleration limits and output cap. Expand
   **AxonTurretTest** for `targetDegrees` and `targetStepDegrees`.
4. Edit a value and use Dashboard's **Save** control to apply it. The running
   controller reads these settings every loop; no rebuild or OpMode restart is
   needed for gains, constraints or targets. Gain changes clear accumulated
   integral error. Profile-constraint or target changes replan from the current
   desired position/velocity. Hardware names take effect at the next INIT.
5. After the setup and zero checks, press START and hold **RT** to follow the
   target. Gain edits can apply while RT remains held; for repeatable comparisons,
   release RT between trials. A target edit while RT is held commands the new
   target. Simply saving a target with RT released cannot start movement.
6. Add a Dashboard graph and select the numeric `turret.*` series listed below.
   If the gamepad changes the target, refresh the configuration pane to read its
   current value; code-side edits do not automatically refresh that pane.

Both classes use `@Config` and public static volatile settings so Dashboard edits
are visible to the control loop. This follows the official
[live configuration guidance](https://acmerobotics.github.io/ftc-dashboard/features).
**Save applies settings to the running app; it does not write your Java source.**
Record the tune and copy it into source as described at the end of this guide.
Changing encoder calibration, gearing, direction or travel limits invalidates
zero and can latch a fault during an active motion request; perform those changes
with motion released and establish zero again.

## What the controller does

The motion profile turns the final angle into intermediate position, velocity
and acceleration setpoints. Long moves accelerate, cruise, then decelerate;
short moves may start slowing down before reaching maximum speed. PID follows
the intermediate position, not an instantaneous jump to the final angle. This
is the standard [profile/setpoint approach](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html).

```text
positionError = profilePosition - measuredPosition
velocityError = profileVelocity - measuredVelocity

feedback = kP * positionError + kI * accumulatedPositionError + kD * velocityError
feedforward = kS * motionDirection + kV * profileVelocity + kA * profileAcceleration
requestedPower = feedback + feedforward
appliedPower = clamp(requestedPower, -maximumPower, +maximumPower)
```

The kS direction follows planned velocity, or planned acceleration at launch.
After the profile ends, kV/kA contributions become zero; kS may still assist
remaining position error. Output becomes zero when final position and speed
are within tolerance, while the dwell timer checks readiness.

`kV` predicts the output needed for speed; `kA` predicts the extra output needed
for acceleration or braking. `kS` compensates for friction. These terms supplement
feedback rather than replace it. The [feedforward reference](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html)
explains the roles of these terms; our output units differ from its motor-voltage
examples.

**Every angle/speed/acceleration here is measured at the TURRET after gearing.**
Output is normalized CR-servo power, not volts. Do not copy flywheel gains, multiply
by battery voltage, or paste WPILib voltage gains directly. An Axon's internal
electronics may make its response different from a bare DC motor; tune the actual
power-to-motion relationship. A useful `kA` may be small or zero.

| Dashboard setting in `AxonTurret` | Meaning / units | Initial value |
| --- | --- | --- |
| `profileEnabled` | Generate smooth setpoints. Leave true for this guide. | true |
| `maxProfileVelocityDegreesPerSecond` | Maximum planned turret speed, deg/s. | 30 |
| `maxProfileAccelerationDegreesPerSecondSquared` | Planned acceleration/braking magnitude, deg/s². | 60 |
| `kS` | Friction compensation, power. | 0 |
| `kV` | Power per deg/s. | 0 |
| `kA` | Power per deg/s². | 0 |
| `kP` | Power per degree of position error. | 0.01 |
| `kD` | Power per deg/s of velocity error. | 0 |
| `kI` | Power per degree-second of accumulated error. | 0 |
| `integralLimit` | Maximum absolute accumulated error, degree-seconds; contribution is `kI * integralLimit`. | 20 |
| `maximumPower` | Cap on the combined feedback + feedforward output. | 0.15 |
| `manualPower` | Power at full stick during LB manual jogging; also capped by `maximumPower`. | 0.10 |

The default profile limits are gentle examples, not measurements of your hardware.
`maximumServoDegreesPerSecond` is a separate **shaft-speed tracking bound**, set
from the model's speed at your supply voltage with margin. Do not lower that bound
to slow the turret; use the profile settings. Planned turret speed cannot exceed
that shaft bound divided by `servoTurnsPerTurretTurn`.

## 1. Establish repeatable test moves

1. Choose the correct robot configuration, INIT, then open Dashboard. `AxonTurret`
   holds the mechanism settings; `AxonTurretTest` holds `targetDegrees` and
   `targetStepDegrees`. Complete the setup guide and press B at physical zero.
2. Keep `profileEnabled=true`, `kI=kD=kS=kV=kA=0`, and P at its starting value.
   Begin with a small target comfortably inside the measured travel limits.
   Hold **RT** to run; release RT to remove power. **X** cancels and requires
   controls to be released before restarting.
3. Start with the default low power cap and profile limits, or reduce them if
   appropriate for your mechanism. A 5–10 degree move is a useful initial check
   only if that range is actually clear. Verify both directions.
4. For each comparison, return to the same starting position and let it stop.
   Release RT before changing gains so each trial has a clear start. Dashboard
   tuning changes reset the integral; gearing/direction/limit calibration changes
   invalidate zero and require the setup procedure again.

Graph these numeric telemetry series in Dashboard:

| Graph together | What to look for |
| --- | --- |
| `turret.goalDeg`, `turret.profileDeg`, `turret.actualDeg` | Final goal, planned ramp and measured following error. |
| `turret.profileDegPerSec`, `turret.actualDegPerSec` | Requested versus measured speed, including the cruise plateau. |
| `turret.profileDegPerSec2` | When acceleration, cruise and braking occur. |
| `turret.feedbackPower`, `turret.feedforwardPower` | How much correction PID must add to the prediction. |
| `turret.requestedPower`, `turret.appliedPower` | Whether output is being clipped; also see `Output limited`. |

Power graphs use the logical positive turret direction, before `servoSign`.
Velocity comes from successive analog readings and can be noisy: judge trends
over several samples, not one spike. In manual mode, use **appliedPower** for
measurements; the feedback/feedforward series describe position control.

## 2. Estimate kS and kV with manual jogging

Keep the mechanism zeroed so manual travel limits apply. Use **LB + left stick X**
and release it before reaching the edge of the usable travel.

- **Friction estimate (`kS`):** slowly increase a small manual command and note the
  lowest applied power that reliably starts movement. Repeat in both directions.
  Use a modest estimate rather than a value that causes a jump. This is a rough
  starting point: breakaway friction and friction while moving can differ.
- **Speed estimate (`kV`):** at several fixed applied powers, record average turret
  speed after it stops accelerating. Use only steady-speed portions with room
  to stop. For one sample, `kV ≈ (abs(power) - kS) / abs(speedDegPerSec)`.
- For two steady points in the same direction, a better slope estimate is
  `kV ≈ (power2 - power1) / (speed2 - speed1)`, using positive magnitudes.
  The intercept `power1 - kV * speed1` estimates friction while moving. Repeat
  both directions and at another speed to see how consistent the model is.

Example arithmetic only: if 0.10 power produces 20 deg/s and the friction estimate
is 0.02, then `kV ≈ (0.10 - 0.02) / 20 = 0.004`. **Do not use these numbers without
measuring them.** If the turret never reaches steady speed within available travel,
use a slower run; don't fit acceleration data as if it were steady motion.

For a long enough profiled move to include a cruise plateau, its distance from
rest must exceed `maxVelocity² / maxAcceleration`. Pick the move only within
your measured range, or lower the profile speed to create a plateau in less space.
Enter the estimated kS/kV and keep kA/I/D zero initially. Repeat the same move
under RT. If actual speed consistently trails the profile during cruise, kV may
be low; if it leads, kV may be high. Check clipping and mechanical binding first.

The manual watchdog requires encoder progress at substantial output. If a
breakaway test faults, release controls, inspect the cause, clear with Y and
re-zero with B. Do not disable the guards to force a bound mechanism to move.

## 3. Tune P, then D

With the approximate feedforward in place, increase P in small steps until actual
position follows the profile and reaches the final goal reliably. Reduce it if
the turret repeatedly crosses the target or oscillates. Use comparable moves
in both directions; increasing P cannot fix a saturated output or wrong gearing.

Add a little D to reduce overshoot and improve speed tracking. In this controller,
D uses **desired velocity minus measured velocity**, so it does not continuously
fight the planned rotation. Too much D amplifies analog-encoder noise and can
make the servo buzz or twitch. Back it down if that happens.

Keep I at zero through these stages. Use the output graphs to ensure feedforward
is doing most of the predictable work while PID supplies corrections. The
[turret-control tuning reference](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-turret.html)
also explains why combined feedforward and feedback helps with moving setpoints.

## 4. Tune kA during the ramps

Keep the same target, speed limit, acceleration limit, kV and PID gains. Inspect
the start and slowdown phases separately from the cruise phase.

1. Start kA at zero. If speed/position lag mainly while accelerating, increase kA
   in small steps. One practical increment is `0.01 / maxAcceleration`: it changes
   the acceleration contribution by 0.01 power at maximum planned acceleration.
   That is a step-size calculation, not a recommended final gain.
2. Repeat forward and reverse moves. Too much kA causes a sharp launch, leading
   the profile, or excessive braking. Reduce it if response gets worse.
3. During slowdown, desired acceleration opposes desired velocity, so the kA
   contribution changes sign. Total power may reverse while the turret still
   rotates forward; that can be correct braking. The watchdog permits this when
   measured motion still follows the profile, while continuing to check progress.
4. If kA does not improve repeatability, leave it at zero. Servo control electronics,
   gearing friction and noisy measurements can make a simple acceleration model
   less useful. Keep gains based on observed improvement.

`kA` uses the **planned acceleration**, not a second derivative of noisy encoder
position. You do not need to calculate measured acceleration to tune it.

## 5. Choose the final profile and use I only if needed

Increase profile speed gradually, then acceleration, repeating the same checks.
Faster speed changes cruise power demand; higher acceleration changes launch and
braking demand. Leave output headroom for PID. A useful rough check is:

```text
kS + kV * maxVelocity + kA * maxAcceleration < maximumPower
```

This estimates the largest positive feedforward during a normal acceleration
phase. The output graph is the actual check. If `Output limited` stays true,
reduce the profile demand or reassess the power cap for the mechanism; don't keep
increasing gains. All terms share the same cap and integral anti-windup accounts
for feedforward saturation.

Only add a small I gain if a repeatable residual position error remains after
friction compensation and P/D tuning. Set `integralLimit` so its contribution
cannot dominate. Reduce I if it creates slow oscillation or delayed reversal.
The integral resets on release, retarget, zero, fault and gain changes.

`toleranceDegrees` and `settledVelocityDegreesPerSecond` define acceptable final
position/speed. `settleTimeMs` requires that condition to persist. At-target status
also requires the profile to finish and encoder data to be fresh. Once settled,
output is zero; a later disturbance outside tolerance resumes correction while
RT stays held. Avoid shrinking tolerance below useful encoder resolution/backlash.

## Troubleshooting and saving the tune

| Symptom | Check / adjustment |
| --- | --- |
| No motion at all | Correct CR firmware/neutral, hardware names, speed bound, reference and direction gates. Read the status message. |
| Stalls just short | Binding/backlash, kS and P; consider I only after those. |
| Lags during cruise | kV estimate or output clipping; reduce planned speed if needed. |
| Tracks cruise but lags on ramps | kA and acceleration demand. |
| Overshoots or oscillates | P/kA too large, D too small, excessive speed, backlash, or insufficient stopping room. |
| Buzzes or twitches | Excessive D/kS, encoder noise, or tolerance too small. |
| Fault after a loop pause | Tracking lost its revolution count; clear the cause, Y, then physical zero with B. |
| Braking-path travel fault | Current motion cannot stop inside the configured profile range; release and reposition manually. Don't widen limits beyond the mechanism. |
| Move timeout | Allowed time is planned duration plus `moveTimeoutMs` (default 4 s extra). Diagnose tracking/gains before increasing the allowance. |

Changing the goal while RT is held replans from the existing desired position and
velocity, so reversal brakes before changing direction. Actual motion can differ
from that plan; software limits are not physical stops. Release/manual mode/STOP
discard the old plan. Re-entering position control starts from the measured state.
`profileEnabled=false` is available for direct-PID comparison: desired velocity and
acceleration become zero, so kV/kA contribute nothing; kS can still assist residual
position correction. Use the profiled mode for the main tune.

Record the final hardware/model, supply configuration, gear ratio, signs, zero
definition, travel limits, six gains, profile limits, output cap and tolerances.
Copy tuned values into the static settings at the top of
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/AxonTurret.java`
and rebuild. Dashboard edits are runtime settings; do not rely on them surviving
an app restart. Test again after restarting and establishing zero. Changes to
servo firmware, PWM range, gearing, load or power supply can require retuning.

## Code verification

The full Java suite passes **246 tests across 32 suites**, including 11 profile
math tests and 15 profile/feedforward integration tests. Live Dashboard gain,
profile-constraint and held-trigger target edits are covered. Both the debug APK and
Sloth bundle build successfully. These checks validate code behavior; this turret
has not been tuned or tested on physical hardware.
