# Pedro 3 and library updates implementation plan

**Goal:** Migrate the 8103 BIOBUZZ SDK 12 project to released Pedro 3 and Ivy, with compatible tooling and tuning.

**Architecture:** Preserve RobotOpMode lifecycle, Ivy ownership, Pinpoint fault handling, and Limelight validation. Replace Pedro 2 APIs with Pedro 3 math, paths, localizer, drivetrain, Foresight, and AutoTune integrations. Foresight requires new robot measurements; autonomous paths remain gated until that configuration is explicitly marked tuned.

**Tech stack:** FTC SDK 12.0.0, Pedro 3.0.0, Ivy core/pedro 1.1.0, AutoTune 1.0.0, Sloth/Load 0.3.0, compatible Dashboard/Panels, FTCLib core.

**Authorization:** User approved Pedro 3 + Ivy migration and requested checks for other library updates. Execute in the migration worktree, then integrate verified changes into ~/Downloads/8103biobuzz.

## Constraints

- Preserve official SDK 12 root build files; adjust team tooling only where new published requirements demand it.
- Keep one Ivy scheduler. Do not add NextFTC or FTCLib's command scheduler.
- Motor output requires a fresh finite READY Pinpoint sample. Cancel, fault, timeout, and STOP immediately clear motion; recovery does not restart an aborted auto/tuner.
- Preserve heading during vision translation correction, real angular velocity, stationary dwell and capture-age gates. BIOBUZZ moving-tag absolute localization stays disabled.
- User clarified that no old values were measured. Treat motor directions as setup examples, reset pod offsets and heading gains to zero, and require actual AutoTune measurements. Foresight calibration values are not interchangeable with Pedro 2.
- No robot deployment. Verify unit tests, tooling tests, APK and Sloth artifacts.

## Task 1: Runtime migration

Files: TeamCode runtime/test Java, especially Constants, Drivetrain, SkeletonAuto, Limelight, and math helpers.

- [x] Verify clean baseline tests on SDK 12.
- [x] Update dependency coordinates to published versions.
- [x] Migrate Pose accessors and Path construction; use Velocity for angular velocity without angle normalization.
- [x] Adapt Constants to MecanumConfig, PinpointConfig, guarded Localizer, ForesightConfig.
- [x] Preserve manual drive output when only sampling localization; synchronously stop Pedro drivetrain on cancel (Follower.stop only changes mode in v3).
- [x] Preserve real path completion rather than Ivy's early parametric completion where sequential actions depend on settling.
- [x] Port behavioral tests, adding tests for v3 stop semantics, completion timing, unconfigured auto, and non-normalized angular velocity.

## Task 2: AutoTune

Files: pedroPathing/Tuning.java, TuningSafety.java, procedures/, and their tests.

- [x] Replace the old copied Pedro 2 selector with registered Pedro 3 AutoTune procedures for mecanum, Pinpoint, Foresight, and path tests.
- [x] Guard motion and cleanup in every procedure, preserving B/STOP/fault latching and temporary-offset restoration.
- [x] Include upstream provenance and license when copying procedure sources.
- [x] Verify lifecycle/safety tests against real v3 APIs.

## Task 3: Other libraries and deployment

Files: TeamCode/build.gradle, tools/run_gradle.py, tools/deploy.py, push scripts if needed; relevant host-side regression tests.

- [x] Audit published versions for FTCLib, Sloth, Load, Slothboard, Panels and test utilities.
- [x] Match Load/Sloth 0.3 deployment protocol and maintain explicit ANDROID_SERIAL targeting.
- [x] Select installed JDK 25 for Load 0.3 builds without changing SDK root pins.
- [x] Validate fake-device Sloth and full APK deployment actions; never contact a robot.

## Task 4: Verify and deliver

Files: START_HERE.md, docs/TUNING.md, docs/NEW_SEASON.md, docs/DEPENDENCIES.md.

- [x] Record exact versions, sources, known limitations and AutoTune steps.
- [x] Run the full Java suite and relevant tooling suite; build APK and Sloth bundle.
- [x] Review diff, inspect dependency graph/APK for duplicate or stale Pedro/Ivy packages.
Delivery: fast-forward the verified commit into `~/Downloads/8103biobuzz`, build there, push the fork and verify the remote commit and clean status.

Verification before delivery: 107 Java tests and 32 host tooling tests passed; APK and Sloth bundle built. APK DEX inspection confirmed Pedro 3, Ivy, Pinpoint, Limelight and FTCLib with no duplicate definitions or old Pedro 2 geometry. Independent runtime review found no critical or important issues; minor review findings were corrected.
