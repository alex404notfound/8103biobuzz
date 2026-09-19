# HIVE orientation, field position and turret auto aim

The Limelight is mounted on the turret. Use its current observation to aim at
the selected alliance's **upward CELL**; use Pinpoint for field-side awareness
and a short prediction between camera observations. Neither sensor alone proves
that a launched ball will clear the opening. This prototype does not choose a
flywheel speed or hood setting, feed a ball, or certify a successful shot.

## Run the tuning OpMode

Select **Hive Auto Aim Test** in the **Prototyping** group. It requires the
drivetrain, healthy Pinpoint, referenced Axon turret and turret-mounted Limelight
3A. It does not initialize the launcher. First complete
[Axon setup](AXON_TURRET_TEST.md) and [turret tuning](AXON_TURRET_TUNING.md).
Use a practice field and the physical Driver Station gamepads.

Open [FTC Dashboard](http://192.168.43.1:8080/dash) on the robot network. The
**HiveAutoAim** configuration contains camera, opening, orientation, vision and
approach settings; **HiveAutoAimTest** contains the starting pose and selected
alliance/CELL. Existing **AxonTurret** settings govern PID/feedforward, profiles,
gearing and travel limits. Dashboard changes apply to the running app; copy a
successful configuration into source to retain it after restart.

| When | Control | Action |
| --- | --- | --- |
| INIT | Gamepad 2 X | Recalibrate Pinpoint while the robot is still. |
| INIT | Gamepad 2 A | Apply the measured start pose, after `startPoseConfigured=true`. |
| After START | Gamepad 1 LB + left-stick X | Low-power manual turret movement for checking/zeroing. |
| After START | Gamepad 1 B | Establish physical turret zero while motion is released. |
| After START | Gamepad 1 Y | Clear a turret fault while motion is released. |
| After START | Hold gamepad 1 RT | Track the selected upward CELL, if all checks pass. |
| After START | Gamepad 1 X | Cancel movement; release motion controls before rearming. |
| After START | Hold gamepad 2 LB + sticks | Drive after the field pose is established; aiming pauses chassis drive. |

Before INIT, place the robot at a measured position and set
`HiveAutoAimTest.startX`, `startY` and `startHeadingDegrees` in the field frame
described below. Only set `startPoseConfigured=true` when those numbers actually
describe that robot placement. Gamepad 2 A applies the pose; the flag by itself
does not reset odometry. Choose the actual alliance and a CELL belonging to it.
If Pinpoint loses readiness after initialization, the field reference is revoked;
stop and reapply a measured pose in INIT before using field-side labels or driving.
Crossing a field divider must never change the alliance. Select the other CELL
when the HIVE tips; this prototype does not pick a new target automatically.

## Measure the geometry before enabling aim

All distances below are **inches**. Turret/chassis axes are forward, left, up.
Tag axes are printed right, printed down, and the pose's tag-normal axis. Camera
optical axes are right, down, forward. Do not mix these frames.

1. Measure `pivotForward` and `pivotLeft` from the robot pose reference point to
   the turret pivot. Set `turretZeroYawDegrees` to the turret's actual physical
   heading relative to the chassis when the turret encoder reads zero. Physical
   forward is normally the easiest zero to reproduce. **Increasing turret encoder
   angle must turn the turret left/counterclockwise when viewed from above.**
   Check that convention using a small manual movement; adjust the Axon encoder
   and output directions together as needed, then repeat zero and PID checks.
   A bench PID can work with either angle convention, but this aim geometry
   specifically requires counterclockwise-positive angles.
2. Measure `cameraForward`, `cameraLeft` and `cameraUp` from that pivot in the
   turret frame. Set `cameraRollDegrees`, `cameraPitchDegrees` and
   `cameraYawDegrees`. Zero rotation means the camera looks turret-forward with
   its image top upward. In this controller, **positive mounting pitch tilts the
   camera downward**, and positive yaw turns it left. Test the displayed geometry
   against known targets before setting `cameraPoseVerified=true`. The candidate
   angle and range diagnostics below remain available with verification flags
   false once a complete geometry is entered.
3. Set `shooterForward`, `shooterLeft` and `shooterYawDegrees` for the muzzle and
   bore relative to the rotating turret frame. These correct horizontal
   parallax; muzzle height and launch elevation are not part of this azimuth
   controller.
4. Measure `openingTagX`, `openingTagY`, `openingTagZ` from the sticker-row center
   to the chosen aperture point, in tag axes. The manual suggests Y=-7.188 for
   the mouth plane, and X=0 for a centered point. Measure/verify the sign and
   value of Z on the actual SDK pose; a wrong normal sign aims outside the CELL.
   `openingNormalTagX/Y/Z` must point **out through the mouth**. Nominally this
   is the printed-up direction (0,-1,0), not the bottom panel's normal. Set
   `openingGeometryVerified=true` only after checking the resulting aim point
   and front/back approach on the real field.
5. Observe both stable positions from several angles, without holding RT. Check
   the displayed CELL pitch and UP/DOWN state. Set `upPitchDegrees`,
   `downPitchDegrees` and `pitchToleranceDegrees` to accommodate verified
   measurements while retaining a clear transition gap. Defaults are +30,
   -30 and an 8-degree tolerance. Then set `cellOrientationVerified=true`.

The three verification flags start false. The opening coordinates and normal
start as editable zero placeholders; the zero-length normal makes the geometry
invalid until configured. Setting the flags is a record of completed physical checks,
not a calibration procedure. Changes invalidate the previous aim observation
and require fresh consistent frames. Do not simply widen tolerances until an
incorrect pose appears acceptable.

Configure a Limelight AprilTag pipeline for **36h11**, **3.25-inch / 0.08255-meter**
tags, with full per-tag 3D pose output. Select that pipeline in the existing
Limelight settings. Validate scale using a measured distance. A fixed-field
MegaTag2 pose from these moving CELL tags is not used by this aim controller.

## Observe first, then move

With the turret referenced, keep RT released and confirm the selected CELL,
tag count, signed pitch, camera-exposure age, horizontal range, approach angle,
and candidate angle. Graph `aim.candidateGoalDeg (unverified)` to inspect the reconstructed
angle during calibration. A finite, nonzero opening normal and finite opening
offsets are required for those diagnostics. The candidate remains **unverified**
until all three flags are set; the commanding `aim.goalDeg` stays unavailable
and holding RT cannot use an unverified candidate. This implementation requires **at least two distinct valid
tags from the selected CELL**. It combines corrected 3D opening estimates and
rejects tag disagreement. At least three fresh, consistent observations spanning
`stableDwellMs` are required before motion; the initial dwell is 200 ms.

Begin with a slow turret profile and a front-facing view. Hold RT briefly and
confirm motion reduces the real horizontal aim error. Releasing RT stops the
turret. An unreachable turret angle is rejected, rather than clamped to a stop
and reported as an aim solution. Lost/expired vision, a downward or tipping CELL,
unhealthy Pinpoint, an unreferenced turret, an invalid configuration, or a
shallow/backside approach also prevents an aiming command.

The chassis must be almost still before the turret follows an aim command.
`HiveAutoAimTest.maxAimChassisSpeed` defaults to 1 inch/second, and
`maxAimChassisTurnDegrees` to 5 degrees/second. These gates supplement the chassis
stop command while holding RT; this prototype does not compensate ball flight
for a moving shooter.

`minimumRangeInches` and `maximumRangeInches` start at 12 and 180.
`minimumApproachDegrees` starts at 30: 90 degrees means directly in front of the
mouth in the horizontal plane, 0 means alongside its plane, and negative means
behind it. This is a conservative geometric filter, **not a ball-clearance or
trajectory model**. Restrict the limits to the actual distances and angles that
have been tested with the shooter.

The controller uses camera exposure time to look up earlier Pinpoint and turret
poses, then computes the target bearing from the current robot and turret-pivot
position. This prevents ordinary camera latency from being treated as a current
measurement. `extraLatencyMs` is only for a measured additional delay; do not
add the Limelight's reported latency a second time. Insufficient pose history
means no aim until samples bracket the exposure.

Blind prediction is **off by default** (`maximumPredictionAgeMs=0`). An optional
value up to 500 ms allows short odometry pre-aim after the image is no longer
current; its age is measured from exposure, not from the moment tracking was
lost. Predicted aim is explicitly labeled and does not confirm that the CELL is
still up. Observed DOWN/TRANSITION states or inconsistent tag geometry cancel
the target. Keep prediction disabled for initial testing and never use it to
authorize feeding.

The field-side center band defaults to 6 inches. It is a manually chosen margin,
not a measured confidence interval reported by Pinpoint. Increase it if observed
drift warrants it, and reestablish the pose at a known location when necessary.

## What the manual establishes

Reviewed against the [BIOBUZZ Competition Manual, TU01](https://ftc-resources.firstinspires.org/ftc/game/manual),
sections 9.6 and 9.9, figures 9-10, 9-11 and 9-15 through 9-17, and
[Team Update 01](https://ftc-resources.firstinspires.org/ftc/game/tu-combined).
The [official assembly guide](https://ftc-resources.firstinspires.org/ftc/field/initialfieldguide),
pages 21, 23 and 31, clarifies the CELL's base, mouth and tag mounting.

| CELL | Tag IDs |
| --- | --- |
| Red, opposite the audience | 30, 31, 32, 33 |
| Red, audience side | 34, 35, 36, 37 |
| Blue, audience side | 38, 39, 40, 41 |
| Blue, opposite the audience | 42, 43, 44, 45 |

IDs identify the CELL, not whether it is currently up. Each CELL carries four
tags in a row on its bottom panel. The printed bottom edge points toward the
field center. The mechanism has two stable states, nominally tilted 30 degrees
either way. Consequently, the printed **up direction of the tag**, transformed
into a level frame, rises on an upward CELL and falls on a downward CELL. The
nominal signed elevations are +30 and -30 degrees. These elevations are a
geometric inference from the mounting and hinge diagrams, not raw Limelight
Euler-angle values specified by FIRST.

Do not classify using `ty`: that is the target's location in the camera image.
Do not compare an uncorrected SDK `getPitch()` directly with 30 degrees either:
camera coordinates and the physical mount change what that number means. Use
the tag's full rotation, the configured camera mounting rotation, and the
printed up axis. A camera with roll or pitch installed incorrectly in software
can reverse or bias the answer. Verify both actual stable positions before
enabling motion. Frames during a tip, ambiguous poses, conflicting tags and
stale observations must not be treated as a confirmed upward target.

This prototype assumes a level chassis and a rigid camera mount. It does not
compensate for the whole robot pitching or rolling over obstacles. Confirm the
state on flat floor with the chassis stationary before aiming.

The tags move with the HIVE. Do not use a fixed field AprilTag map containing
their startup positions to overwrite Pinpoint as the HIVE tips. Direct relative
aiming does not need that assumption. A future absolute localization method
would need a surveyed model of both HIVE states and sufficient state confidence.

## Which side of the field is the robot on?

Pinpoint integrates movement from a starting pose. It does not independently
discover the field origin or alliance. Establish a measured robot pose before
using side labels or field-coordinate targeting. Rebooting or resetting odometry
does not establish a known physical location by itself.

This project's planning frame has the audience at the bottom, red at the left,
+X to the right, +Y away from the audience, and counterclockwise headings from
+X. In the nominal 144-inch square, x=72 divides left and right; y=72 divides
audience and far halves. The center band reports uncertainty near those
boundaries instead of forcing a left/right or audience/far choice. Side labels describe the
robot's location; the alliance selected before START stays the same when the
robot crosses the center. The uploaded field image is rotated relative to this
frame, so its screen left/right must not be copied directly into coordinates.

Odometry drift matters most near a dividing line or after long driving. Prefer
fresh relative vision for final alignment. A last-seen target projected through
recent Pinpoint motion is only a short-lived prediction; it must expire if the
camera loses the goal. That prediction is especially limited here because the
goal itself can move while unseen.

## Midpoint of two AprilTags

The desired point is in the opening, not on the sticker. For an upright sticker,
the four tag centers are at lateral offsets **-6.5, -2.75, +2.75 and +6.5 inches**
from the cluster center. The inside pair and outside pair each straddle that
center. Two adjacent tags do not. An arithmetic mean of their horizontal image
angles also does not produce the physical midpoint under perspective.

Estimate each tag's 3D pose, subtract its known lateral offset in the tag frame,
then combine the estimates of the same CELL center. Use only consistent tags
from that CELL and frame. One tag could mathematically provide an estimate using
its known offset; this implementation requires two for a consistency check.
Do not average tags belonging to opposite CELLS or different alliances.

Figure 9-15 puts the reference-hole line 9.938 inches behind the mouth and the
tag-center line 2.75 inches ahead of the reference holes. Their difference is
**7.188 inches from tag center toward the mouth**, along the printed up axis.
An aim point also needs a chosen height into the aperture, measured from the
base in the direction opposite the bottom panel's outward normal. Keep the
camera-to-turret pivot translation, mounting yaw/pitch/roll, turret encoder zero,
and muzzle offset in the geometry. Treating camera bearing as muzzle bearing
without these offsets is most noticeable close to the goal.

## Why shallow shots can miss even with correct horizontal aim

Figure 9-11 shows a house-shaped opening: approximately 20 inches wide, 14 inches
to its peak, and 7.61 inches to the upper corners of the vertical walls. The
CELL is about 12 inches deep. Its full bounding rectangle is not all open space.
POLLEN is about 2.8 inches in diameter and NECTAR about 3.6 inches, with actual
size and shape variation.

Think about the path of the **ball's center**, with clearance for the ball radius
plus the observed shot spread. At a shallow approach, an edge or sidewall can
intercept that path even when the camera points at the apparent center. Aiming
closer to one side may improve one edge's clearance while making another worse.
Moving the chassis to a better angle is often the more repeatable solution.

For an idealized opening polygon with corners (-10,0), (10,0), (10,7.61),
(0,14), (-10,7.61) inches, the point maximizing static distance to its edges is
roughly (0,6.4). This is a mathematical starting point, **not an official aim
point or a ballistic solution**. Real shots have an entry direction, a rising or
falling trajectory, spin, deformation, and front-to-back clearance requirements.
Even an accurately aimed point does not guarantee those conditions.

Begin with a conservative angle/range window and measured shooter presets.
Record actual shots for each ball type, distance and approach angle. Add a small
bounded lateral bias only when those trials support it. Reject difficult angles
rather than extrapolating an untested bias. A later shot planner can intersect a
calibrated trajectory with the full tilted aperture and internal walls; the
current aiming result should not be used as automatic permission to feed.
