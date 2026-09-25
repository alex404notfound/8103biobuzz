# BIOBUZZ software requirements

Reviewed September 18, 2026 against the **2026-2027 Competition Manual, TU01**
and **Team Update 01, September 17**. This is an implementation plan based on the
manual and this checkout. [Shooter Tuning](SHOOTER_TUNING.md) supplies two-motor
speed tuning and a positional-servo control for practice. Vision is tested
separately in [Hive Auto Aim Test](HIVE_AUTO_AIM.md); the full game routines below
are still planned.

Primary references: [current complete manual](https://ftc-resources.firstinspires.org/ftc/game/manual),
[team updates](https://ftc-resources.firstinspires.org/ftc/game/tu-combined), and
[official field resources](https://ftc-resources.firstinspires.org/ftc/field).
Page numbers below refer to the complete TU01 manual. Some separate chapter
downloads still carry V1; apply the team updates when consulting them.

## What this project already provides

Pedro supplies path following, Ivy coordinates commands, and Pinpoint supplies
odometry. Pedro Quickstart is useful for tuning these capabilities; game rules,
mechanism control, and strategy remain team code.

| Current code | What still needs work |
| --- | --- |
| `Robot` creates the drivetrain, optional Limelight, and one example motor. | Add the actual intake, storage/feed, launcher, and flower mechanism after hardware is chosen. |
| `RobotOpMode` owns scheduling, sensor updates, and shutdown. | Add match-phase permissions. Its INIT loop also executes commands and subsystem updates. |
| `SkeletonAuto` follows one example straight path. | Build measured game routes, scoring commands, deadlines, and a park fallback. |
| `SkeletonTeleOp` drives and locks headings. | Add mechanism controls, inventory display, aiming assistance, and cancellation. Lock alliance selection before play; the current live bumper overrides are examples. |
| `Limelight` handles optional fixed-tag pose correction. | Add a separate observation path for moving hive targets. Keep game tags out of the existing absolute-pose correction. |
| [Brushland sensor tests](BRUSHLAND_COLOR_TEST.md) configure and calibrate the sensor, then classify colors through digital bulk reads. | Bench-validate thresholds and wiring, then integrate classification with the intake and counting pipeline. |
| `FieldConstants` has nominal dimensions and an unmeasured start placeholder. | Add surveyed landmarks, robot clearance, approach poses, and explicit tag-to-cell identities. |

## First existing-code issue: competition telemetry

`Robot` currently constructs `MultipleTelemetry` with Driver Station and Dashboard.
Panels remains available for AutoTune but is not a normal robot telemetry sink.
R704.C-D restricts match network use, including additional logging
and streaming services and continuous video. Introduce an explicit competition
configuration with Driver Station telemetry; disable the Dashboard/Panel
services and any tuning/video streams for matches. Merely removing a telemetry
sink does not establish that an independently started library service is off.
Verify the actual service lifecycle on the Control Hub. Keep tuning tools in the
practice configuration, and disconnect programming computers before match play.
See [construction rules](https://ftc-resources.firstinspires.org/ftc/game/manual-12),
R704, TU01 pp. 138-139.

## Game constraints to encode

These are compact rule references, not an alternative rulebook.

| Manual reference | Constraint |
| --- | --- |
| 10.4; G403-G404 | AUTO 30 seconds, transition 8 seconds, TELEOP 120 seconds; no powered movement during transition or after TELEOP. |
| G304-G305 | Start on the correct half, touching perimeter, outside loading zone/flowers, motionless after INIT, contacting four pollen; enable the DS AUTO timer. |
| G407-G409 | Four controlled pieces total; no opponent nectar; no catching/deflecting hive spills before another contact. |
| G410 | Nectar cannot enter a flower before the final 60 seconds; the field timer is authoritative. |
| G417-G418 | Induce tips by launching into the upward cell. Flower entry is through the top; only pollen exits through the bottom. |
| G426-G427 | Own-alliance tips release human nectar; all remaining nectar unlocks at 60 seconds. Introduced nectar must hit the loading-zone tile before the robot. |
| R102, R105 | Start within 18 x 18 x 18 inches; expanded envelope 18 x 24 x 29 inches, physically constrained. |
| R503 | At most eight motors and eight servos across event configurations. |

Sources: [game details](https://ftc-resources.firstinspires.org/ftc/game/manual-10),
[game rules](https://ftc-resources.firstinspires.org/ftc/game/manual-11), and
[construction rules](https://ftc-resources.firstinspires.org/ftc/game/manual-12).

## Required software behavior for our scoring robot

### Match state and command permissions

Create a `MatchContext` with alliance, phase, timing confidence, and action
permissions. Keep initialization for autonomous separate from initialization
during the transition into TeleOp: pre-match positioning is allowed under R103,
but automatically homing a servo during the transition can cause movement.

An OpMode's `getRuntime()` is not the field clock. TeleOp can be started during
the transition or late. Use an explicit driver start/synchronization procedure,
visible phase status, and conservative timing. For flower deposits, require a
driver confirmation of the field's final-minute cue as well as the phase gate.
Drivers must still stop at the actual match end.

Recommended initial policy: gate all flower **deposits** until the final minute.
G410 specifically names nectar, while 10.5.2 describes flower scoring more
broadly. This conservative policy must not disable legal bottom pollen retrieval.
Keep the distinction visible in the implementation instead of treating it as a
blanket ban on interacting with flowers.

Use nonblocking Ivy commands with subsystem ownership, deadlines, and explicit
cleanup on cancellation. Preserve the existing `Robot.stop()` guarantee when
adding each actuator. A match stop must never begin an automatic retract or
homing movement.

### Intake, inventory, and feeding

Represent pieces as `POLLEN`, `OWN_NECTAR`, `OPPONENT_NECTAR`, or `UNKNOWN`, using
the alliance chosen in INIT. Count the complete transport path, including the
intake throat and feeder, rather than only hopper slots. Track uncertainty and
debounce sensor transitions; sensor bounce must not manufacture another piece.

Inhibit pickup at capacity. Design classification/rejection at the entrance to
avoid deliberately acquiring opponent nectar. On an accidental acquisition,
stop and promptly release it locally; do not route it through storage or scoring.
Ambiguous sensor readings should request recovery rather than assume a legal
piece. Hardware guards remain necessary for pieces software cannot see.

Add bounded jam recovery and an operator inventory correction with visible
feedback. Feeding should decrement inventory on confirmed passage, not on a
button press. Measure separate handling settings for pollen and nectar.
Floor collection after a dump and loading-zone collection both need approach
procedures that avoid intercepting airborne pieces.

### Launcher and hive observations

Model hive observations as `UNKNOWN`, `AUDIENCE_CELL_UP`, `FAR_CELL_UP`, and
`TIPPING`, with timestamps and confidence. Use tag identity **and** observed
orientation/motion; seeing an ID alone does not prove that cell is upward.

Start with measured shooting stations and a calibrated launcher preset for each
piece type. A flywheel design would need velocity control and a ready-speed
check; a different launcher needs its own readiness feedback. Do not assume a
turret, adjustable hood, or flywheel exists until the mechanism is selected.

The command sequence should acquire the correct alliance target, aim, wait for
launcher readiness, and feed under feedback. Stop feeding when tipping begins,
wait for a stable observation, then reacquire the newly upward cell. Timeouts,
lost targets, and inconsistent observations must inhibit automatic feeding.
Do not infer a tip from the number of shots: partner actions and current cell
contents also affect it. Any software tip count is an estimate, not the official
score or automatic permission for the human player to introduce nectar.

Keep Pinpoint/Pedro as the field-navigation baseline. Add camera-to-launcher
calibration and relative aiming separately from `Limelight.relocalize()`.

### Flower and garden commands

Provide separate actions for bottom pollen pickup, top deposit, and garden
release. A flower command should align, confirm the intended piece and phase,
move the selected mechanism, confirm release, and retract while the match is
active. Use measured positions and feedback; do not invent servo endpoints.

Keep queue order available to the operator so they can choose when to spend
nectar. Avoid automatic opponent-nectar removal or a recovery motion that pulls
pieces through the flower side. Software position limits supplement physical
stops; they cannot satisfy R105.B by themselves.

## Scoring priorities and match workflow

The scoring table gives 20 points per tip; leave is 3 in AUTO and park is 5 in
either period. Final cell pieces and pieces in an owned flower are 2 each;
garden pieces are 1. Bottom nectar earns 5, while topmost nectar determines
flower ownership. Standard-event RP thresholds are 16 leave/park points and
4/7 tips; regional/championship thresholds remain TBA and Premier events may
choose thresholds. See 10.5 and Tables 10-2/10-3 in
[game details](https://ftc-resources.firstinspires.org/ftc/game/manual-10), pp. 86-91.

The following is a proposed strategy, not a required robot design:

| Stage | Proposed behavior |
| --- | --- |
| First reliable autonomous | Leave, travel to the alliance loading zone, and park partially inside it while remaining clear of the perimeter wall at AUTO end. Coordinate the footprint with the partner. |
| Scoring autonomous | From a measured legal start, leave, use held preloads at a verified shooting station, then park before the deadline. Add a collection cycle only after timing measurements justify it. |
| Early TeleOp | Collect legal floor/bottom-flower pollen and cycle the hive. Collect released nectar after it lands. Preserve manual driving and a clear cancel command. |
| Final minute | Allow flower deposits and ownership changes; show remaining time and piece order. Select hive cycles, flower work, or garden delivery according to observed opportunities. |
| Closing seconds | Reserve measured travel/settling time for parking. Stop starting cycles that cannot finish. |

Two successful leave-plus-AUTO-park routines would total 16 points for the
alliance, making this a useful initial reliability target. This is arithmetic
from the scoring table, not a claim that this checkout already performs it.
Plan AUTO paths within the alliance half and coordinate partner lanes to reduce
interference risk under G402; that rule prohibits opponent interference rather
than defining an unconditional center-line crossing ban.
Use flower ownership observations as advisory: another robot can change the
state. Avoid presenting an estimated inventory or score as official.

## Field information available now

Keep nominal field geometry separate from measured robot poses and launcher
calibration. Use CAD and the actual practice field for target transforms and
clearance, not pixels measured from an illustration.

| Nominal item | Value |
| --- | --- |
| Field | 144 x 144 inches |
| Flower top opening | 4-inch diameter; 21.5 inches above tiles |
| Upward hive opening, vertical extent | Bottom 53.5 inches; top 65.6 inches |
| Bottom of hive above tiles | **30.6 inches**, corrected in TU01; this is not the shooting target height |
| Pollen / nectar diameter | About 2.8 / 3.6 inches |
| AprilTags | 36h11; 3.25-inch tag size |

| Cell identity | Tag IDs |
| --- | --- |
| Red, opposite audience | 30-33 |
| Red, audience side | 34-37 |
| Blue, audience side | 38-41 |
| Blue, opposite audience | 42-45 |

Verified visually against Figures 9-10, 9-12, and 9-17, and section 9.9 in the
[manual](https://ftc-resources.firstinspires.org/ftc/game/manual), pp. 71-77;
the hive clearance correction is in [TU01](https://ftc-resources.firstinspires.org/ftc/game/tu-combined).
These tag groups identify moving cells, not stationary field landmarks.

The existing field convention is audience at the bottom, red on the left,
origin at bottom-left, inches, and counterclockwise heading from +X. Blue
geometry uses a 180-degree rotation. Add loading/garden polygons, flower
approaches, hive obstacles, and shooting stations in that convention. Account
for the robot's entire footprint and extensions when checking clearance or park
overlap. Keep start poses and shot presets unconfigured until measured.

## Implementation order and acceptance checks

1. **Competition foundation:** service configuration, match phase permissions,
   INIT alliance selection, and a measured leave/park auto. Verify no third-party
   match streams, no transition movement, and immediate actuator shutdown.
2. **Game model and intake:** typed inventory with entrance/exit feedback and
   bounded recovery. Test mixed four-piece capacity, wrong color, unknown color,
   bouncing/stuck sensors, reverse flow, and interrupted feeds.
3. **Hive scoring:** launcher control, cell observations, and feed coordination.
   Test both alliances and both stable states, tipping, stale/occluded tags,
   missed shots, partner-induced tips, and cancellation while spinning/feeding.
4. **Flowers and full autonomous:** measured mechanism presets and timed routes.
   Test early/late TeleOp starts, the 60-second boundary, independent pollen
   retrieval, jam/time-limit fallback, and actual park clearance with a partner.

Use a fake clock for timing tests and recorded sensor observations for game-state
tests. Bench and field tests must establish actuator positions, launcher
readiness, trajectory margins, repeatability, and worst-case cycle time before
enabling automatic scoring. The manual cannot supply motor names, wiring,
ratios, servo travel, camera mounting, or this robot's start pose.

The original review added documentation; the launcher prototype is described
separately above. Build validation does not establish hardware performance or
verify the future game behaviors in this plan.
