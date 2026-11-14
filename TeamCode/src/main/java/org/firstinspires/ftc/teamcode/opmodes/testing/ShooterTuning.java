package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.field.Canvas;
import com.bylazar.field.Circle;
import com.bylazar.field.Line;
import com.bylazar.field.Style;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.math.ShotTable;
import org.firstinspires.ftc.teamcode.hardware.BulkCacheSession;
import org.firstinspires.ftc.teamcode.telemetry.RobotTelemetry;
import org.firstinspires.ftc.teamcode.robot.FieldConstants.HiveCell;
import org.firstinspires.ftc.teamcode.robot.ShotTableStore;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Launcher.Ball;

import java.io.IOException;
import java.util.EnumMap;
import java.util.Locale;
import java.util.Map;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

/** Move, tune, save, then interpolate RPM from field distance using separate big/small-ball tables. */
@Configurable
@Disabled
@TeleOp(name = "Shooter Tuning", group = "Prototyping")
public class ShooterTuning extends OpMode {
    public static volatile Ball selectedBall = Ball.SMALL_POLLEN;
    public static volatile HiveCell selectedCell = HiveCell.RED_AUDIENCE;
    public static volatile boolean automaticLookup = false;
    public static volatile double rpmStep = 50;
    public static volatile double startX = 72, startY = 24, startHeadingDegrees = 90;
    public static volatile double driveScale = 0.35;
    private static final long STATIONARY_DWELL_NANOS = 300_000_000L;

    private final LongSupplier clock;
    private final Supplier<Telemetry> panelsTelemetry;
    private Drivetrain drive;
    private Launcher launcher;
    private ShotTableStore store;
    private final BulkCacheSession bulkCache = new BulkCacheSession();
    private Map<Ball, ShotTable> tables = ShotTableStore.empty(), undo;
    private boolean active, stopped, releaseRequired = true, fieldPoseInitialized, tablesLoaded;
    private Ball ball;
    private HiveCell cell;
    private boolean lookupMode;
    private long stationarySince = Long.MIN_VALUE;
    private double range = Double.NaN;
    private ShotTable.Lookup lookup = new ShotTable().lookup(Double.NaN);
    private String saveStatus = "No sample saved this run";
    private String poseStatus = "INIT: enter measured start pose, then GP2 A";
    private String runStatus = "INIT";

    public ShooterTuning() {
        this(null, null, null, System::nanoTime, RobotTelemetry::browserTelemetry);
    }

    ShooterTuning(Drivetrain drive, Launcher launcher, ShotTableStore store, LongSupplier clock,
                  Supplier<Telemetry> panelsTelemetry) {
        this.drive = drive; this.launcher = launcher; this.store = store;
        this.clock = clock; this.panelsTelemetry = panelsTelemetry;
    }

    @Override public void init() {
        Scheduler.reset();
        active = stopped = fieldPoseInitialized = tablesLoaded = automaticLookup = false;
        releaseRequired = true; undo = null; stationarySince = Long.MIN_VALUE;
        telemetry = new JoinedTelemetry(telemetry, panelsTelemetry.get());
        telemetry.setMsTransmissionInterval(100);
        try {
            bulkCache.enable(hardwareMap);
            if (drive == null) drive = new Drivetrain(hardwareMap, telemetry);
            if (launcher == null) launcher = new Launcher(hardwareMap, telemetry);
            if (store == null) store = new ShotTableStore(AppUtil.getInstance().getSettingsFile(ShotTableStore.FILE_NAME));
            drive.usePreviousStartingPose();
            drive.setFieldOverlay(this::drawCalibration);
            selectConfiguration();
            try {
                tables = store.load();
                tablesLoaded = true;
                saveStatus = "Loaded saved tables; A saves, B undoes the last save";
            } catch (IOException failure) {
                tables = ShotTableStore.empty();
                saveStatus = "LOAD FAILED - file preserved; saving/lookup disabled: " + failure.getMessage();
            }
        } catch (RuntimeException failure) { shutdown(failure); }
    }

    @Override public void init_loop() {
        if (stopped) return;
        try {
            boolean calibrate = gamepad2.xWasPressed(), seed = gamepad2.aWasPressed();
            selectConfiguration();
            if (calibrate) {
                fieldPoseInitialized = false;
                poseStatus = "Calibrating: keep still, then GP2 A to reapply measured pose";
                drive.recalibrateLocalization();
            }
            sampleDrive();
            if (seed && !calibrate) {
                Pose proposed = new Pose(startX, startY, Math.toRadians(startHeadingDegrees));
                if (ShotTable.validPose(proposed) && drive.isLocalizationReady()) {
                    drive.setStartingPose(proposed); fieldPoseInitialized = true;
                    stationarySince = Long.MIN_VALUE;
                    poseStatus = "Measured field pose applied";
                } else {
                    fieldPoseInitialized = false;
                    poseStatus = "Pose refused: use finite in-field coordinates and wait for Pinpoint READY";
                }
            }
            launcher.periodic();
            updateLookup();
            report();
        } catch (RuntimeException failure) { shutdown(failure); }
    }

    @Override public void start() {
        if (stopped) return;
        try {
            active = true; releaseRequired = true; automaticLookup = lookupMode = false;
            gamepad1.resetEdgeDetection(); gamepad2.resetEdgeDetection();
            stopOutputs();
            launcher.enable();
        } catch (RuntimeException failure) { shutdown(failure); }
    }

    @Override public void loop() {
        if (!active || stopped) return;
        try {
            boolean save = gamepad1.aWasPressed(), revert = gamepad1.bWasPressed();
            if (gamepad1.dpadLeftWasPressed()) selectedBall = Ball.SMALL_POLLEN;
            if (gamepad1.dpadRightWasPressed()) selectedBall = Ball.LARGE_NECTAR;
            if (gamepad1.yWasPressed()) automaticLookup = !automaticLookup;
            boolean up = gamepad1.dpadUpWasPressed(), down = gamepad1.dpadDownWasPressed();
            selectConfiguration();
            if (!lookupMode && Double.isFinite(rpmStep) && rpmStep > 0 && rpmStep <= 500) {
                if (up) launcher.adjustRpm(rpmStep);
                if (down) launcher.adjustRpm(-rpmStep);
            }
            // These controls only reset odometry during INIT.
            gamepad2.aWasPressed(); gamepad2.xWasPressed();
            if (gamepad1.x) releaseRequired = true;
            if (releaseRequired || (!spinHeld() && !gamepad1.left_bumper)) launcher.idle();
            if (releaseRequired || !gamepad2.left_bumper) drive.stop();

            sampleDrive();
            updateLookup();
            if (releaseRequired) {
                stopOutputs();
                runStatus = "Release RT, both LBs and X to arm";
                if (controlsReleased()) releaseRequired = false;
            } else if (ball == null || cell == null) {
                stopOutputs(); runStatus = "Select a ball and CELL";
            } else {
                // A deliberate low-voltage check powers the single shooter.
                if (gamepad1.left_bumper && !spinHeld()) {
                    launcher.requestMotorTest();
                    runStatus = "Shooter direction check";
                } else if (spinHeld() && !gamepad1.left_bumper) {
                    double rpm = lookupMode ? lookup.rpm : launcher.getManualRpm();
                    if (!lookupMode || (tablesLoaded && fieldUsable() && lookup.usable)) {
                        launcher.requestRpm(rpm);
                        runStatus = lookupMode ? "Automatic distance RPM" : "Manual RPM";
                    } else {
                        launcher.idle(); runStatus = !fieldUsable() ? "Field pose unavailable" : lookup.status;
                    }
                } else {
                    launcher.idle(); runStatus = "Motor stopped";
                }
                if (gamepad2.left_bumper && fieldUsable() && Double.isFinite(driveScale)
                        && driveScale > 0 && driveScale <= 0.5) {
                    double forward = -gamepad2.left_stick_y, left = -gamepad2.left_stick_x;
                    double turn = -gamepad2.right_stick_x;
                    // Cap the combined mecanum wheel demand, including diagonal movement plus turning.
                    double scale = driveScale / Math.max(1, Math.abs(forward) + Math.abs(left) + Math.abs(turn));
                    drive.driveRobotRelative(forward * scale, left * scale, turn * scale);
                } else drive.stop();
            }
            launcher.periodic();
            // Save only the target that has actually settled on this cycle, never a spin-up reading.
            if (save) savePoint();
            else if (revert) undoSave();
            report();
        } catch (RuntimeException failure) { shutdown(failure); }
    }

    private void selectConfiguration() {
        Ball nextBall = selectedBall;
        HiveCell nextCell = selectedCell;
        boolean nextLookup = automaticLookup;
        if (ball != nextBall || cell != nextCell || lookupMode != nextLookup) {
            stopOutputs();
            releaseRequired = true;
            stationarySince = Long.MIN_VALUE;
        }
        ball = nextBall; cell = nextCell; lookupMode = nextLookup;
        if (ball != null) launcher.selectBall(ball);
    }

    private void sampleDrive() {
        bulkCache.clear();
        drive.periodic();
        if (fieldPoseInitialized && !drive.isLocalizationReady()) {
            fieldPoseInitialized = false; releaseRequired = true;
            poseStatus = "Localization interrupted: STOP/INIT and reapply measured field pose";
            stopOutputs();
        }
        Velocity velocity = drive.getVelocity();
        boolean still = fieldUsable() && velocity != null && Double.isFinite(velocity.vx)
                && Double.isFinite(velocity.vy) && Double.isFinite(velocity.omega)
                && Math.hypot(velocity.vx, velocity.vy) <= 1 && Math.abs(velocity.omega) <= Math.toRadians(5);
        if (!still) stationarySince = Long.MIN_VALUE;
        else if (stationarySince == Long.MIN_VALUE) stationarySince = clock.getAsLong();
    }

    private boolean fieldUsable() {
        return fieldPoseInitialized && drive.isLocalizationReady() && ShotTable.validPose(drive.getPose()) && cell != null;
    }
    private boolean stationary() {
        return stationarySince != Long.MIN_VALUE && clock.getAsLong() - stationarySince >= STATIONARY_DWELL_NANOS;
    }
    private boolean spinHeld() { return gamepad1.right_trigger > 0.25; }
    private boolean controlsReleased() {
        return !spinHeld() && !gamepad1.left_bumper && !gamepad2.left_bumper && !gamepad1.x;
    }
    private ShotTable table() { return ball == null ? new ShotTable() : tables.get(ball); }

    private void updateLookup() {
        range = fieldUsable() ? ShotTable.range(drive.getPose(), cell) : Double.NaN;
        lookup = table().lookup(range);
    }

    private void savePoint() {
        if (!tablesLoaded) { saveStatus = "Save disabled: resolve calibration load failure first"; return; }
        if (lookupMode) { saveStatus = "Switch to MANUAL before saving a measured RPM"; return; }
        if (releaseRequired || ball == null || !fieldUsable() || !stationary() || gamepad2.left_bumper) {
            saveStatus = "Save refused: stop driving, hold still 0.3 s, and use a healthy measured field pose"; return;
        }
        if (!spinHeld() || gamepad1.left_bumper || !launcher.isAtSpeed()
                || launcher.getTargetRpm() != launcher.getManualRpm()) {
            saveStatus = "Save refused: hold RT until the shooter is at the tuned RPM"; return;
        }
        ShotTable.Point point;
        Map<Ball, ShotTable> next = new EnumMap<>(tables);
        try {
            point = new ShotTable.Point(drive.getPose(), cell, launcher.getTargetRpm());
            next.put(ball, table().with(point));
        }
        catch (IllegalArgumentException invalid) { saveStatus = invalid.getMessage(); return; }
        // File sync can block. Stop all motors before touching storage and require a fresh hold afterward.
        stopOutputs(); releaseRequired = true;
        try {
            store.save(next);
            undo = tables; tables = next;
            saveStatus = String.format(Locale.US, "SAVED %s: %.1f in -> %.0f RPM (%d points). Release RT to continue.",
                    ball, point.distance, point.rpm, table().points().size());
            updateLookup();
        } catch (IOException failure) {
            saveStatus = "SAVE FAILED - current table unchanged: " + failure.getMessage();
        }
    }

    private void undoSave() {
        if (!tablesLoaded || undo == null) { saveStatus = "No save from this run to undo"; return; }
        if (!controlsReleased()) { saveStatus = "Release shooter/drive controls before undo"; return; }
        stopOutputs(); releaseRequired = true;
        try {
            store.save(undo);
            tables = undo; undo = null; updateLookup();
            saveStatus = "Last save undone on the Control Hub";
        } catch (IOException failure) { saveStatus = "UNDO FAILED - current table unchanged: " + failure.getMessage(); }
    }

    private void drawCalibration(Canvas canvas) {
        if (cell == null || ball == null) return;
        Style targetStyle = new Style("transparent", "#fbbf24", .2);
        Style savedShotStyle = new Style("#fbbf24", "transparent", 0);
        double x = ShotTable.targetX(cell), y = ShotTable.targetY(cell);
        canvas.getItems().add(new Circle(x, y, 4, targetStyle));
        canvas.getItems().add(new Line(x - 5, y, x + 5, y, targetStyle));
        canvas.getItems().add(new Line(x, y - 5, x, y + 5, targetStyle));
        if (fieldUsable()) canvas.getItems().add(new Line(drive.getPose().x(), drive.getPose().y(), x, y, targetStyle));
        for (ShotTable.Point point : table().points()) {
            if (point.cell == cell) canvas.getItems().add(new Circle(point.x, point.y, 1.5, savedShotStyle));
        }
    }

    private void report() {
        telemetry.addData("Calibration ball / mode / CELL", ball + " / " + (lookupMode ? "LOOKUP" : "MANUAL") + " / " + cell);
        telemetry.addData("Calibration storage", saveStatus);
        telemetry.addData("Field reference", poseStatus);
        telemetry.addData("shooter.rangeInches", range);
        telemetry.addData("shooter.lookupRpm", lookup.rpm);
        telemetry.addData("shooter.lowerDistance", lookup.lowerDistance);
        telemetry.addData("shooter.upperDistance", lookup.upperDistance);
        telemetry.addData("shooter.interpolationFraction", lookup.fraction);
        telemetry.addData("Lookup status", tablesLoaded ? lookup.status : "Calibration file unavailable");
        telemetry.addData("Run status", runStatus);
        telemetry.addData("Manual target RPM", launcher.getManualRpm());
        telemetry.addData("Stationary / RPM settled", stationary() + " / " + launcher.isAtSpeed());
        StringBuilder points = new StringBuilder();
        for (ShotTable.Point point : table().points())
            points.append(String.format(Locale.US, "%.1f in: %.0f RPM | ", point.distance, point.rpm));
        telemetry.addData("Saved distances (selected ball)", points.length() == 0 ? "None" : points.toString());
        telemetry.addLine("GP1 left/right: small/big | up/down: RPM | RT: run selected | LB: direction test");
        telemetry.addLine("GP1 A: save settled manual point | B: undo last save | Y: manual/lookup | X: cancel");
        telemetry.addLine("GP2 LB + sticks: slow robot-relative drive | INIT GP2 A: measured start | X: calibrate");
        telemetry.addLine("Panels Launcher: separate big/small gains and manual RPM. Aim/feed manually.");
        telemetry.update();
    }

    private void stopOutputs() {
        try { if (drive != null) drive.stop(); }
        finally { if (launcher != null) launcher.idle(); }
    }
    private void shutdown(RuntimeException failure) {
        try { stop(); } catch (RuntimeException stopFailure) { failure.addSuppressed(stopFailure); }
        throw failure;
    }
    @Override public void stop() {
        if (stopped) return;
        stopped = true; active = false; automaticLookup = false;
        try { if (drive != null) { drive.setFieldOverlay(null); drive.stop(); } }
        finally {
            try { if (launcher != null) launcher.stop(); }
            finally {
                try { bulkCache.close(); }
                finally { Scheduler.reset(); }
            }
        }
    }
}
