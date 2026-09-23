# Two-motor shooter flywheel: FTC Dashboard tuning

Run **Shooter Flywheel Tuning** in the **Prototyping** group. This motor-only OpMode
needs the two shooter motors and **one encoder cable**, plus the Hub's battery
voltage reading. It does not require a hood servo, drivetrain, turret or camera.

Both motors drive **one flywheel through a belt**. They must have equal effective
gearing. The **left motor's encoder (`launcherLeft`) is hard-coded** to measure
the shared speed; a single PID +
feedforward controller sends the same logical power to both motors. The other
motor's encoder is never read and can remain unplugged.

The design follows your [WPILib flywheel tuning reference](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html):
control **speed**, tune feedforward first, then add feedback for correction and
recovery after a shot. The example's numeric gains are not applicable to our
motor-RPM units. kA is useful here only while the requested speed is ramping;
it contributes zero once the speed reference is constant.

## Dashboard and robot setup

1. Install the full APK for the first run of the new OpMode/configuration. Configure
   two **goBILDA 6000 RPM (1:1)** motors named `launcherLeft` and `launcherRight`.
   The custom motor type and controller use **28 encoder counts per motor shaft
   revolution**. These are motor RPM values; external flywheel gearing changes
   wheel RPM. The example `teamconfig8103_flywheel_bench.xml` puts them on Control
   Hub motor ports 0/1; match your actual wiring. Connect the encoder cable from
   `launcherLeft` to that motor's corresponding Hub encoder port. Leave the
   `launcherRight` encoder unplugged; its motor power cable remains connected.
2. Connect to the robot network. For a Control Hub, open
   [FTC Dashboard](http://192.168.43.1:8080/dash). For an Android phone Robot
   Controller, the usual address is [this Dashboard address](http://192.168.49.1:8080/dash).
   These are the [official Dashboard connection addresses](https://acmerobotics.github.io/ftc-dashboard/gettingstarted).
3. In Dashboard's **Config** view, expand **ShooterFlywheel** for gains, direction
   verification, ramp rate, voltage cap and readiness settings. Expand
   **ShooterFlywheelTuning** for `targetRpm` and `characterizationVoltage`.
   Edit a value and press **Save** to apply it. Gains and setpoints are live;
   no rebuild is needed between tuning trials. The fields use `@Config` and
   `public static volatile`, following [Dashboard's configuration guidance](https://acmerobotics.github.io/ftc-dashboard/features).
4. Select **Shooter Flywheel Tuning**, INIT, and inspect battery voltage,
   the fixed **LEFT** feedback label and the single `flywheel.measuredRpm` value.
   Encoder selection is not a Dashboard setting; changing it requires a code edit.
   INIT never requests motor motion. START, then release all motion
   buttons once to arm the controls. Use the Driver Station gamepad for held
   motion commands while editing settings in Dashboard.

Hardware names and `leftReversed`/`rightReversed` are applied at INIT.
After changing a direction, STOP and INIT again. Changing a direction during
operation invalidates readiness and stops the controller.

## Controls and direction verification

| Gamepad 1 | Action |
| --- | --- |
| LB + X | Hold a low-voltage left-motor check. |
| LB + Y | Hold a low-voltage right-motor check. |
| LB + A | Hold shared open-loop voltage from `characterizationVoltage`. |
| RT | Hold closed-loop motor RPM from `targetRpm`. |
| D-pad up/down | Change target by `rpmStep` (default 100 RPM). |
| D-pad left/right | Change characterization request by `voltageStep` (default 0.1 V). |
| X without LB | Cancel; release controls before restarting. |
| Release motion controls / STOP | Remove power immediately and coast. |

Individual checks use `ShooterFlywheel.testVoltage`, initially **1 V**, limited to
1.8 V and the overall voltage cap. The unpowered motor is set to FLOAT. Because
the shafts are linked, **either motor must turn the left encoder**.
Keep the belt installed for both checks; the feedback source does not switch
when you change which motor is powered.

Check each motor briefly. Each must drive the mechanism in the same desired
launching direction, and the left encoder must report positive RPM during
either check. Change the appropriate reversed setting and re-INIT if needed.
Set `directionsVerified=true` only after checking. Shared-voltage and RPM modes
remain blocked until then. Positive RPM does not by itself prove the ball will
be launched in the correct physical direction; inspect the mechanism too.

## Controller, units and defaults

The controller uses `RUN_WITHOUT_ENCODER` for output while still reading encoder
velocity. It computes software PID + feedforward, then calls `setPower()` on both
motors. There is no additional Hub `setVelocity()` loop or REV PIDF layer to tune.

```text
measuredRpm = leftMotorEncoderTicksPerSecond * 60 / 28
referenceRpm = ramp toward targetRpm at maxAccelerationRpmPerSecond
referenceAcceleration = change in referenceRpm / elapsed seconds

feedforwardVolts = kS + kV * referenceRpm + kA * referenceAcceleration
feedbackVolts = kP * (referenceRpm - measuredRpm)
              + kI * integralOfRpmError
              - kD * measuredRpmChangePerSecond
requestedVolts = feedforwardVolts + feedbackVolts
appliedVolts = clamp(requestedVolts, 0, min(maxVoltage, batteryVolts))
bothMotorPower = appliedVolts / batteryVolts
```

kS is zero at a zero reference. Target zero and release bypass the ramp and
remove power immediately. When reducing a positive RPM target, the reference
ramps downward, but output never reverses: the wheel coasts if requested voltage
falls below zero. Actual slowdown can therefore take longer than the reference.

| `ShooterFlywheel` field | Units / purpose | Initial value |
| --- | --- | --- |
| `kS` | Volts to compensate for friction. | 0 |
| `kV` | Volts per motor RPM. | 0.002: rough 12 V / 6000 RPM estimate, not a measurement |
| `kA` | Volts per RPM/s of reference acceleration. | 0 |
| `kP` | Volts per RPM of speed error. | 0.002 |
| `kI` | Volts per RPM-second of integrated error. | 0 |
| `kD` | Volts per RPM/s of measured acceleration; derivative on measurement avoids target-step kick. | 0 |
| `maxAccelerationRpmPerSecond` | Rate limit for positive target changes and reductions. | 1000 RPM/s |
| `maxVoltage` | Cap on the combined output of feedback and feedforward. | 3 V for initial bench work |
| `integralLimitRpmSeconds` | Bound on accumulated error; maximum I contribution is `kI * integralLimitRpmSeconds`. | 5000 |
| `rpmTolerance`, `speedDwellMs` | Measured shared speed must stay within final-target tolerance for this duration. | 150 RPM, 250 ms |

These flywheel gains use **volts and motor RPM**, while the Axon turret uses
normalized servo power and turret degrees. Do not transfer gains between them.
Measured battery voltage converts desired voltage into a duty-cycle estimate;
it is not a separate measurement of motor terminal voltage under load.

## Tuning sequence

### 1. Graph RPM and voltage

In Dashboard, select numeric telemetry fields and use **Graph**. Display:

- `flywheel.targetRpm`, `flywheel.referenceRpm`, `flywheel.measuredRpm`.
- `flywheel.referenceRpmPerSec`, `flywheel.feedforwardVolts`, `flywheel.feedbackVolts`.
- `flywheel.requestedVolts`, `flywheel.appliedVolts`, `flywheel.batteryVolts`.

`Feedback encoder` identifies the source of that single RPM reading. `Output
limited / at speed` reports clipping and final-target readiness. There are no
left/right RPM, average RPM or encoder-difference graphs.

### 2. Measure steady-speed feedforward

Begin unloaded, with room for the wheel to coast. Use **LB+A** at several small
positive `characterizationVoltage` values inside `maxVoltage`. Once speed has
stabilized, record measured RPM and **appliedVolts**, not a request that was clipped.
Repeat each point. Stop if the feedback is missing, the mechanism binds, or the
power path behaves unexpectedly.

For two steady-speed points:

```text
kV ≈ (voltage2 - voltage1) / (rpm2 - rpm1)
kS ≈ voltage1 - kV * rpm1
```

Use several points to check that a straight-line approximation is reasonable.
Do not use spin-up samples in this calculation. A slow voltage increase until
movement begins gives a rough friction estimate, but breakaway friction can
differ from the steady-speed intercept. Do not force a negative kS into the
controller; recheck the data, measurement range and units.

Example arithmetic only: 2.0 V at 900 RPM and 3.0 V at 1400 RPM give
`kV ≈ 0.002 V/RPM` and `kS ≈ 0.2 V`. These are hypothetical measurements, not
values established for your robot.

### 3. Verify feedforward, then tune P

Enter measured kS/kV; keep I/D/kA zero. For a brief feedforward-only check, set P
to zero and hold RT at a reachable target. Steady RPM should approach the target.
If it misses consistently, revisit kV/kS and clipping before adding much feedback.

Then increase P in small steps. P should correct remaining speed error and help
the flywheel recover after a shot. Reduce P if speed or motor power oscillates.
Repeat at several intended shooting speeds. The initial 3 V cap may make higher
speeds unreachable; raise the cap only as appropriate for the assembled mechanism
and within its ratings, or use lower targets during initial tuning. More gain
cannot overcome a voltage cap.

Once unloaded behavior is repeatable, test controlled single-ball shots through
the completed mechanism. Look for the shared RPM dip and time until speed
regains readiness. This OpMode does not feed balls automatically.

### 4. Tune kA only for changing speed

Keep kA zero until steady-speed tracking is good. If spin-up lags the ramp, add
a little kA. An increment of `0.1 / maxAccelerationRpmPerSecond` changes its
contribution by 0.1 V at full ramp acceleration. This calculates a step size, not
a final gain. Reduce kA if spin-up overshoots or output becomes unnecessarily
sharp. kA should return to zero contribution at constant reference RPM.

If the reference demands more acceleration than the mechanism can deliver,
reduce `maxAccelerationRpmPerSecond`. Leave voltage headroom for recovery:

```text
kS + kV * intendedRpm + kA * maxAccelerationRpmPerSecond < usable voltage cap
```

Check the applied/requested voltage graphs rather than relying only on this
estimate. Decreasing speed may require coasting; negative kA contribution does
not enable reverse braking.

### 5. I/D, readiness and recovery

Leave I and D at zero unless a specific remaining problem justifies them. A small
I can remove repeatable residual error after the feedforward fit is good; too much
can cause slow recovery or oscillation. D can amplify velocity noise and is often
unnecessary for a steady flywheel. Gain edits reset accumulated integral, and
anti-windup uses the total feedforward-plus-feedback voltage limits.

Readiness requires the ramp to reach the **final** target, the left encoder's
RPM inside tolerance, fresh samples, and the dwell time. Choose tolerance from measured
shot consistency and recovery behavior, not just to make the ready flag turn on.

## Faults, launcher integration and saving

- Power of at least 0.5 V without sufficient left-encoder RPM triggers the
  response watchdog and stops both motors. This also applies to either individual
  motor test through the belt. Unexpected negative RPM in paired modes faults.
- A single encoder cannot independently detect a failed second motor or all belt
  failures. Verify the linkage and each motor's contribution during bench checks.
- Once the ramp reaches final RPM, `spinupTimeoutMs` allows 4 seconds by default
  to reach/recover speed. A deliberate new ramp gets its own ramp time; encoder
  response checks remain active throughout target changes.
- Invalid sensor/battery data or an active-loop gap over `maximumSampleGapMs`
  stops output. Release motion controls to acknowledge a fault and diagnose its
  status before retrying. STOP is terminal for that OpMode instance.

**Launcher Prototype uses this exact `ShooterFlywheel` controller too.** Tuning
`ShooterFlywheel` changes its motor behavior; `Launcher` now contains the ball RPM
presets and hood settings. The old `Launcher.velocityP/I/D/F` fields were removed
so there is only one set of motor gains. The standalone tuner does not require a
hood; the full launcher keeps its hood settling/coasting checks. Speed readiness
alone never authorizes an automatic shot.

Copy the measured gains, directions, cap, ramp rate and tolerances into the static
fields in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/ShooterFlywheel.java`,
then rebuild. Set shooting presets in `Launcher.smallBallRpm` / `largeBallRpm`.
Dashboard Save applies values live; it does not edit Java source for a future
restart. Repeat checks after restarting, with the actual launcher load and power
configuration.

The turret remains independently tunable in **Axon Turret Test**, using Dashboard
sections **AxonTurret** and **AxonTurretTest**. Its complete instructions are in
[AXON_TURRET_TUNING.md](AXON_TURRET_TUNING.md).

## Code verification

The full suite passes **246 Java tests across 32 suites**, including 28 linked
flywheel controller tests and 5 flywheel tuning OpMode tests. The debug APK and
Sloth bundle both build successfully. Tests cover voltage compensation, shared
output, RPM conversion, ramp/feedforward behavior, live edits, both-encoder
readiness, faults and cleanup. Motor behavior and final gains still need testing
on the actual robot; no hardware was connected or deployed to during this work.
