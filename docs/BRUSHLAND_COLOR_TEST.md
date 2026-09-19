# Brushland Color Rangefinder: BIOBUZZ setup and bulk-read test

BIOBUZZ 2026-27 uses **yellow POLLEN, red NECTAR, and blue NECTAR**. The current
TU01 manual identifies these in section 9.8, page 74. It specifies piece colors,
not sensor hue thresholds. [Official game manual](https://ftc-resources.firstinspires.org/file/ftc/game/manual).

This adds three standalone OpModes, with no motors or drivetrain required:

| Driver Station name | Purpose | Required hardware names |
| --- | --- | --- |
| Brushland Configure | Save output mode and thresholds once over I2C | `Color` as REV Color Sensor V3 |
| Brushland Hue Check | Measure internal hue to choose thresholds | `colorHue` as Analog Input |
| Brushland Color Test | Test digital classification and loop timing | `colorPin0`, `colorPin1` as Digital Device |

The configuration code is in
[`ConfigureColorRangefinder.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/ConfigureColorRangefinder.java).
The reusable reader is
[`BrushlandColorSensor.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/BrushlandColorSensor.java).

## Digital encoding

Our configuration makes sensor pin 0 match blue OR yellow, and sensor pin 1 match
red OR yellow. Both require the piece to be within `maxDistanceMm`.

| Sensor pin 0 | Sensor pin 1 | Raw result |
| --- | --- | --- |
| false | false | `NONE_OR_UNKNOWN` |
| true | false | `BLUE_NECTAR` |
| false | true | `RED_NECTAR` |
| true | true | `POLLEN` (yellow) |

The reader requires the same result for 40 ms before accepting a color. A change
immediately invalidates the previous color during this settling period. The test
also labels own/opponent nectar using an alliance selected in INIT.

Two bits have no spare fault/presence state: **00 is not proof of an empty intake**.
An unrecognized object or an out-of-range ball also produces 00. Disconnected or
floating inputs can imitate valid colors, including yellow; this test cannot
diagnose wiring failure. It classifies a sample and does not count inventory.

## 1. Program the sensor

Connect the sensor to a hub **I2C** port. In a separate sensor configuration, set
that bus to **REV Color Sensor V3**, name **`Color`**, and activate it. Use the
sensor's digital/I2C switch position if your revision has a switch.

Before every programming attempt, reset the sensor to I2C. For current hardware:
unplug it, INIT and STOP **Brushland Configure**, then reconnect to I2C while the
SDK is retrying communication. The communication warning should clear and the LED
blink. Older pre-November-2024 units use the reset button instead: hold until the
LED blinks off, power-cycle, and put the switch toward the connector.

Select **Brushland Configure**, press INIT, then set its fields in Dashboard
before START (or edit the source constants and rebuild). START writes once;
confirm the two sensor LED blinks, press STOP, and reconnect to the appropriate
port. Both pins are always programmed. Configuration persists across power loss;
repeated runs must begin with a reset because threshold writes add ranges.
[Brushland programming/reset reference](https://docs.brushlandlabs.com/sensors/color-rangefinder/configuration).

| Setting | Initial value | Meaning |
| --- | --- | --- |
| `outputMode` | `DIGITAL_BIOBUZZ` | Use `ANALOG_HUE_CALIBRATION` for step 2 |
| `blueMinDegrees`, `blueMaxDegrees` | 180, 250 | Provisional blue hue interval |
| `yellowMinDegrees`, `yellowMaxDegrees` | 55, 90 | Provisional yellow hue interval |
| `redMinDegrees`, `redMaxDegrees` | 0, 50 | Provisional red hue interval |
| `maxDistanceMm` | 20 | Shared proximity gate; allowed 1..100 mm |

Those intervals are starting points from the manufacturer's older three-color
example, **not a BIOBUZZ calibration**. This code deliberately leaves LED brightness
unchanged. Keep the same illumination while calibrating and testing.

## 2. Measure your actual game pieces

1. Reset to I2C and program `outputMode = ANALOG_HUE_CALIBRATION`.
2. STOP, unplug, and connect sensor pin 0's output to a hub **Analog Input**. On
   switch-equipped units, move the switch away from the connector for analog mode.
   Configure the receiving analog channel as `colorHue`; activate that configuration.
   [`teamconfig8103_color_hue.xml`](../TeamCode/src/main/res/xml/teamconfig8103_color_hue.xml)
   is a channel-0 example; verify your revision's output routing.
3. Run **Brushland Hue Check**. Place one ball at the actual mounted sensing gap.
   Hold **A** to record its hue while rotating/repositioning it. Release A before
   removing the ball; **B** clears the sample range. Record bounds for all three
   colors under expected lighting and at the nearest/farthest working positions.
4. Choose intervals with a small margin around the observed values, leaving gaps
   between colors. If readings overlap, improve the mounting/lighting before
   widening intervals. Use the sensor's measured hue; screen RGB colors and nominal
   textbook hue values are not a substitute for this measurement.
5. Red may straddle zero: for example, `redMinDegrees = 340`,
   `redMaxDegrees = 30` encodes 340..360 OR 0..30. Never use 0..360 just because a
   min/max capture spans the seam. Watch the live readings to find both clusters.
6. Reset to I2C again and restore the digital/I2C switch position. Program
   `DIGITAL_BIOBUZZ` with your measured intervals. Set `maxDistanceMm` just beyond
   the farthest intended ball surface while excluding the background. Validate
   this gate with real balls and an empty intake.

The hue reader uses `voltage / 3.3 * 360` to measure the firmware's internal hue;
the configuration converts degrees to the sensor's threshold scale. Runtime
digital mode returns only the two decisions, not RGB or numeric distance.
[Brushland reading reference](https://docs.brushlandlabs.com/sensors/color-rangefinder/reading-data).

Dashboard changes are tuning values: record the final values in source and
reprogram the sensor after changing them. Editing runtime Java or Dashboard
thresholds alone does not replace the configuration already stored in the sensor.

## 3. Wire and run the digital test

After digital programming, STOP and move the sensor cable to one hub **Digital**
connector. Configure **both channels** of that connector as **Digital Device**,
using `colorPin0` and `colorPin1`. Deactivate/remove the I2C sensor entry in this
runtime configuration so the hub does not keep trying to poll it.

On REV connectors the blue wire is the even channel and the white wire is the odd
channel. Brushland's Color Rangefinder pages do not establish which wire is its
logical pin 0, so **do not equate the sensor's pin number with the hub channel
number**. [REV digital wiring reference](https://docs.revrobotics.com/duo-control/sensors/digital).

Start with
[`teamconfig8103_color_digital.xml`](../TeamCode/src/main/res/xml/teamconfig8103_color_digital.xml)
for connector 0/1, or assign the two names in the DS. Its name-to-channel mapping
is provisional. Run **Brushland Color Test** and verify known red and blue balls:
red must show `0 / 1`, blue `1 / 0`, yellow `1 / 1`. If red and blue are exactly
reversed, swap the **hardware names** of the two channels and reactivate. If they
do not produce clean opposing signals, check calibration, distance, mode and
wiring first. Both digital channels remain INPUT; never drive them as outputs.

In INIT, D-pad **left selects RED**, **right selects BLUE**; START locks the
selection. Watch raw bits, raw/stable classification, and timing. Move each color
through the sensing position repeatedly; rotate it, check empty background and
just-outside-distance behavior, and repeat at transport speed. A ball must remain
recognizable long enough for the 40 ms debounce; reduce that value only after
testing the speed/noise tradeoff.

## Why these reads save loop time

The digital test puts hubs in `MANUAL` bulk-caching mode, clears each cache once
at the start of each INIT/RUN cycle, and then reads each pin once. Both channels
on the same hub share one bulk response. Telemetry reads saved values and is
updated every 100 ms. There is no sensor I2C read, sleep, or persistent write in
the runtime loop. Bulk reads still take time; displayed timing includes the
standalone hub fetch and does not promise zero latency or a particular speedup.

For later integration into this project's robot loop:

```java
// Create once during initialization, with the two digital names in the robot config.
BrushlandColorSensor colorSensor = new BrushlandColorSensor(hardwareMap);

// Once each loop AFTER RobotOpMode's existing robot.clearBulkCache():
colorSensor.update();
BrushlandColorSensor.PieceColor piece = colorSensor.getStableColor();
boolean opponentNectar = colorSensor.isOpponentNectar(Alliance.current);
```

Import `org.firstinspires.ftc.teamcode.hardware.BrushlandColorSensor` and
`org.firstinspires.ftc.teamcode.robot.Alliance`. The shared reader does not clear
caches itself. Do not add another cache clear between the drivetrain and color
reads. `RobotOpMode` already owns that operation. The new standalone tests restore
the hubs' previous cache modes on STOP.

## Build and verification

Build with `python3 tools/run_gradle.py :TeamCode:testDebugUnitTest :TeamCode:assembleDebug`.
The unit tests cover pin decoding, dwell time/bounce, cached getters, hub clear/read
ordering, alliance locking, and persistent threshold packets/validation.
Software tests cannot verify a physical sensor, the channel mapping, hue bounds,
or sensing reliability at transport speed; complete the bench procedure above.
