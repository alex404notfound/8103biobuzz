package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.BulkCacheSession;
import org.firstinspires.ftc.teamcode.telemetry.RobotTelemetry;
import org.firstinspires.ftc.teamcode.math.FieldSideEstimator;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.FieldConstants.HiveCell;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.HiveAutoAim;
import org.firstinspires.ftc.teamcode.subsystems.ServoTurret;

/** Pinpoint-only azimuth test. No camera, launcher or feeder is constructed. */
@Configurable
@Disabled
@TeleOp(name = "Hive Auto Aim Test", group = "Prototyping")
public class HiveAutoAimTest extends OpMode {
    public static volatile boolean useSmallTurret = false;
    public static volatile Alliance alliance = Alliance.RED;
    public static volatile HiveCell selectedCell = HiveCell.RED_AUDIENCE;
    public static volatile boolean startPoseConfigured = false;
    public static volatile double startX = 0, startY = 0, startHeadingDegrees = 0;
    public static volatile double sideUncertaintyInches = 6;
    public static volatile double maxAimChassisSpeed = 1, maxAimChassisTurnDegrees = 5;
    private ServoTurret turret;
    private Drivetrain drive;
    private HiveAutoAim aim;
    private final BulkCacheSession bulkCache = new BulkCacheSession();
    private boolean active, stopped, releaseRequired, fieldPoseInitialized;
    private String poseStatus = "Field origin unknown: measure start pose, then press GP2 Cross in INIT";

    public HiveAutoAimTest() { }
    HiveAutoAimTest(ServoTurret turret, Drivetrain drive, HiveAutoAim aim, Telemetry telemetry) {
        this.turret = turret; this.drive = drive; this.aim = aim; this.telemetry = telemetry;
    }

    @Override public void init() {
        Scheduler.reset();
        active = false; stopped = false; releaseRequired = true; fieldPoseInitialized = false;
        telemetry = RobotTelemetry.forOpMode(telemetry);
        telemetry.setMsTransmissionInterval(50);
        try {
            bulkCache.enable(hardwareMap);
            turret = new ServoTurret(hardwareMap, useSmallTurret ? ServoTurret.small : ServoTurret.big);
            drive = new Drivetrain(hardwareMap, telemetry);
            aim = new HiveAutoAim();
        } catch (RuntimeException failure) { shutdown(failure); }
    }

    @Override public void init_loop() {
        if (stopped) return;
        try {
            boolean seed = gamepad2.crossWasPressed(), calibrate = gamepad2.squareWasPressed();
            selectCell();
            if (calibrate) {
                fieldPoseInitialized = false;
                poseStatus = "Pinpoint calibration requested: reapply measured field pose";
                drive.recalibrateLocalization(); aim.clear();
            }
            sample();
            if (seed) seedFieldPose();
            report();
            telemetry.addLine("In INIT, GP2 Square calibrates Pinpoint while still and GP2 Cross applies the measured start pose.");
            telemetry.update();
        } catch (RuntimeException failure) { shutdown(failure); }
    }

    private void seedFieldPose() {
        if (!startPoseConfigured || !Double.isFinite(startX) || !Double.isFinite(startY)
                || !Double.isFinite(startHeadingDegrees) || startX < 0 || startX > 144 || startY < 0 || startY > 144
                || !drive.isLocalizationReady()) {
            fieldPoseInitialized = false;
            aim.clear();
            poseStatus = "Start pose refused: configure measured inches/heading and wait for Pinpoint READY";
            return;
        }
        drive.setStartingPose(new Pose(startX, startY, Math.toRadians(startHeadingDegrees)));
        aim.clear(); fieldPoseInitialized = true;
        poseStatus = "Measured field pose applied (odometry can still drift)";
    }

    @Override public void start() {
        if (stopped) return;
        active = true; releaseRequired = true;
        gamepad1.resetEdgeDetection(); gamepad2.resetEdgeDetection();
    }

    @Override public void loop() {
        if (!active || stopped) return;
        try {
            boolean jogLeft = gamepad1.dpadLeftWasPressed(), jogRight = gamepad1.dpadRightWasPressed();
            selectCell();
            // Consume these INIT-only edges even while active; never queue a later pose reset.
            gamepad2.crossWasPressed(); gamepad2.squareWasPressed();
            if (gamepad1.square) releaseRequired = true;
            boolean aimHeld = gamepad1.right_trigger > 0.25;
            if (releaseRequired || aimHeld || !gamepad2.left_bumper) drive.stop();
            sample();
            if (releaseRequired) {
                aim.clear();
                if (!aimHeld && !gamepad1.left_bumper && !gamepad1.square && !gamepad2.left_bumper)
                    releaseRequired = false;
            } else {
                if (gamepad1.left_bumper) {
                    if (jogLeft != jogRight) turret.nudgeDegrees(jogLeft ? -5 : 5);
                } else if (aimHeld && fieldPoseInitialized && chassisStationary()) {
                    double goal = aim.getTurretGoalDegrees();
                    if (Double.isFinite(goal)) turret.setAngleDegrees(goal);
                }
                // Hold-to-aim owns a stopped chassis; manual drive needs its own deadman and known field frame.
                if (!aimHeld && gamepad2.left_bumper) {
                    if (fieldPoseInitialized && alliance != null)
                        drive.arcadeDrive(-gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.right_stick_x, alliance);
                    else drive.stop();
                }
            }
            report(); telemetry.update();
        } catch (RuntimeException failure) { shutdown(failure); }
    }

    private void selectCell() {
        boolean far = gamepad1.dpadUpWasPressed(), audience = gamepad1.dpadDownWasPressed();
        if (alliance == null || (!far && !audience)) return;
        HiveCell next = alliance == Alliance.RED
                ? (audience ? HiveCell.RED_AUDIENCE : HiveCell.RED_FAR)
                : (audience ? HiveCell.BLUE_AUDIENCE : HiveCell.BLUE_FAR);
        if (selectedCell != next) { selectedCell = next; aim.clear(); }
    }
    private boolean chassisStationary() {
        Velocity v = drive.getVelocity();
        return drive.isLocalizationReady() && Double.isFinite(maxAimChassisSpeed) && maxAimChassisSpeed >= 0
                && maxAimChassisSpeed <= 3 && Double.isFinite(maxAimChassisTurnDegrees)
                && maxAimChassisTurnDegrees >= 0 && maxAimChassisTurnDegrees <= 15
                && Math.hypot(v.vx, v.vy) <= maxAimChassisSpeed
                && Math.abs(Math.toDegrees(v.omega)) <= maxAimChassisTurnDegrees;
    }
    private void sample() {
        bulkCache.clear();
        drive.periodic();
        if (fieldPoseInitialized && !drive.isLocalizationReady()) {
            fieldPoseInitialized = false;
            poseStatus = "Pinpoint interrupted: reapply measured field pose in INIT";
        }
        Pose p = drive.getPose();
        double commanded = turret.getCommandedDegrees();
        aim.update(fieldPoseInitialized, drive.isLocalizationReady(), p.x(), p.y(), p.heading(),
                Double.isFinite(commanded) ? commanded : 0, turret.getMinDegrees(), turret.getMaxDegrees(),
                alliance, selectedCell);
    }
    private void report() {
        Pose p = drive.getPose();
        FieldSideEstimator.Result side = FieldSideEstimator.estimate(p.x(), p.y(), fieldPoseInitialized,
                drive.isLocalizationReady(), sideUncertaintyInches);
        telemetry.addData("Field reference", poseStatus);
        telemetry.addData("Field half (estimate)", side.lateral + " / " + side.longitudinal);
        telemetry.addData("Alliance / selected CELL", alliance + " / " + selectedCell);
        telemetry.addData("Aim source", "Pinpoint + nominal raised-opening map");
        telemetry.addData("Aim status", aim.getStatus());
        telemetry.addData("Turret calibration", turret.isCalibrated() ? "Ready" : "Set servo sweep, zero and safe positions in Panels before INIT");
        telemetry.addData("aim.targetXInches", aim.getTargetXInches());
        telemetry.addData("aim.targetYInches", aim.getTargetYInches());
        telemetry.addData("aim.fieldBearingDeg", aim.getFieldBearingDegrees());
        telemetry.addData("aim.approachDeg", aim.getApproachDegrees());
        telemetry.addData("aim.rangeInches", aim.getRangeInches());
        telemetry.addData("aim.goalDeg", aim.getTurretGoalDegrees());
        telemetry.addData("aim.candidateGoalDeg", aim.getCandidateGoalDegrees());
        telemetry.addData("turret.commandedDeg", turret.getCommandedDegrees());
        telemetry.addData("turret.commandedPosition", turret.getCommandedPosition());
        telemetry.addLine("Hold GP1 L1 and press D-pad left/right to jog. Hold R2 to aim. Square cancels.");
        telemetry.addLine("Hold GP2 L1 and use the sticks to drive after setting the field pose. Holding GP1 R2 stops the chassis.");
        telemetry.addLine("GP1 D-pad down: audience CELL | up: far CELL. Driver selects the raised end.");
        if (releaseRequired) telemetry.addLine("Release GP1 R2, L1, and Square plus GP2 L1 to arm.");
    }
    private void shutdown(RuntimeException failure) {
        try { stop(); } catch (RuntimeException stopFailure) { failure.addSuppressed(stopFailure); }
        throw failure;
    }
    @Override public void stop() {
        if (stopped) return;
        stopped = true; active = false;
        try { if (drive != null) drive.stop(); }
        finally {
            try { if (aim != null) aim.clear(); }
            finally {
                try { bulkCache.close(); }
                finally { Scheduler.reset(); }
            }
        }
    }
}
