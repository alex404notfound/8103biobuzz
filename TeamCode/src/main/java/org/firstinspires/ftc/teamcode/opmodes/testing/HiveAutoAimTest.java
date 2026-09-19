package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.math.FieldSideEstimator;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.FieldConstants.HiveCell;
import org.firstinspires.ftc.teamcode.subsystems.AxonTurret;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.HiveAutoAim;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

import java.util.Collections;
import java.util.List;

/** Practice azimuth test. Does not construct or run the launcher or feeder. */
@Config
@TeleOp(name = "Hive Auto Aim Test", group = "Prototyping")
public class HiveAutoAimTest extends OpMode {
    public static volatile Alliance alliance = Alliance.RED;
    public static volatile HiveCell selectedCell = HiveCell.RED_AUDIENCE;
    public static volatile boolean startPoseConfigured = false;
    public static volatile double startX = 0, startY = 0, startHeadingDegrees = 0;
    public static volatile double sideUncertaintyInches = 6;
    public static volatile double maxAimChassisSpeed = 1, maxAimChassisTurnDegrees = 5;
    private AxonTurret turret;
    private Drivetrain drive;
    private Limelight vision;
    private HiveAutoAim aim;
    private List<LynxModule> hubs = Collections.emptyList();
    private boolean active, stopped, releaseRequired, fieldPoseInitialized;
    private String poseStatus = "Field origin unknown: measure start pose, then INIT gamepad2 A";

    public HiveAutoAimTest() { }
    HiveAutoAimTest(AxonTurret turret, Drivetrain drive, Limelight vision, HiveAutoAim aim, Telemetry telemetry) {
        this.turret = turret; this.drive = drive; this.vision = vision; this.aim = aim; this.telemetry = telemetry;
    }

    @Override public void init() {
        Scheduler.reset();
        active = false; stopped = false; releaseRequired = true; fieldPoseInitialized = false;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);
        try {
            hubs = hardwareMap.getAll(LynxModule.class);
            for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            turret = new AxonTurret(hardwareMap);
            drive = new Drivetrain(hardwareMap, telemetry);
            // Never feed chassis heading into MegaTag2 for a rotating camera or moving game tags.
            vision = new Limelight(hardwareMap, telemetry);
            aim = new HiveAutoAim(Limelight.pipelineIndex);
        } catch (RuntimeException failure) { shutdown(failure); }
    }

    @Override public void init_loop() {
        if (stopped) return;
        try {
            boolean seed = gamepad2.aWasPressed(), calibrate = gamepad2.xWasPressed();
            if (calibrate) { drive.recalibrateLocalization(); aim.clear(); }
            sample();
            if (seed) seedFieldPose();
            report();
            telemetry.addLine("INIT GP2 X: stationary Pinpoint calibration | A: apply measured starting pose");
            telemetry.update();
        } catch (RuntimeException failure) { shutdown(failure); }
    }

    private void seedFieldPose() {
        if (!startPoseConfigured || !Double.isFinite(startX) || !Double.isFinite(startY)
                || !Double.isFinite(startHeadingDegrees) || startX < 0 || startX > 144 || startY < 0 || startY > 144
                || !drive.isLocalizationReady()) {
            fieldPoseInitialized = false;
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
        turret.enable();
    }

    @Override public void loop() {
        if (!active || stopped) return;
        try {
            boolean zero = gamepad1.bWasPressed(), clear = gamepad1.yWasPressed();
            // Consume these INIT-only edges even while active; never queue a later pose reset.
            gamepad2.aWasPressed(); gamepad2.xWasPressed();
            if (gamepad1.x) releaseRequired = true;
            // Release/cancel removes servo power before sampling any further hardware.
            if (releaseRequired || motionReleased()
                    || (!gamepad1.left_bumper && gamepad1.right_trigger > 0.25 && !aim.hasAim())) turret.idle();
            if (releaseRequired || gamepad1.right_trigger > 0.25 || !gamepad2.left_bumper) drive.stop();
            // zeroHere is a request consumed by periodic. Issue it after idle and before sample,
            // otherwise the next released loop would cancel the request before it is processed.
            if (!releaseRequired && motionReleased()) {
                if (clear) { turret.clearFault(); aim.clear(); }
                if (zero) { turret.zeroHere(); aim.clear(); }
            }
            sample();
            if (releaseRequired) {
                turret.idle(); drive.stop(); aim.clear();
                if (motionReleased() && !gamepad1.x && !gamepad1.b && !gamepad1.y && !gamepad2.left_bumper)
                    releaseRequired = false;
            } else {
                if (gamepad1.left_bumper) {
                    turret.requestManual(Math.abs(gamepad1.left_stick_x) < 0.1 ? 0 : gamepad1.left_stick_x);
                } else if (gamepad1.right_trigger > 0.25) {
                    if (chassisStationary() && aim.hasAim()) turret.requestPosition(aim.getTurretGoalDegrees());
                    else turret.idle();
                } else turret.idle();
                // Hold-to-aim owns a stopped chassis; manual drive needs its own deadman and known field frame.
                if (gamepad1.right_trigger > 0.25 || !gamepad2.left_bumper || !fieldPoseInitialized
                        || alliance == null) drive.stop();
                else drive.arcadeDrive(-gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.right_stick_x, alliance);
            }
            report(); telemetry.update();
        } catch (RuntimeException failure) { shutdown(failure); }
    }

    private boolean motionReleased() {
        return !gamepad1.left_bumper && gamepad1.right_trigger <= 0.25 && Math.abs(gamepad1.left_stick_x) < 0.1;
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
        for (LynxModule hub : hubs) hub.clearBulkCache();
        // Apply previous loop's bounded request while reading this loop's encoder sample.
        turret.periodic(); drive.periodic(); vision.periodic();
        if (fieldPoseInitialized && !drive.isLocalizationReady()) {
            fieldPoseInitialized = false;
            poseStatus = "Pinpoint interrupted: reapply measured field pose in INIT";
        }
        Pose p = drive.getPose();
        LLResult frame = vision.getFreshResult();
        aim.update(frame, vision.getFreshResultAgeMs(), drive.isLocalizationReady(), p.x(), p.y(), p.heading(),
                turret.hasReference(), turret.getPositionDegrees(), alliance, selectedCell);
    }
    private void report() {
        Pose p = drive.getPose();
        FieldSideEstimator.Result side = FieldSideEstimator.estimate(p.x(), p.y(), fieldPoseInitialized,
                drive.isLocalizationReady(), sideUncertaintyInches);
        telemetry.addData("Field reference", poseStatus);
        telemetry.addData("Field half (estimate)", side.lateral + " / " + side.longitudinal);
        telemetry.addData("Alliance / selected CELL", alliance + " / " + selectedCell);
        telemetry.addData("Cell / tags", aim.getCellState() + " / " + aim.getTagCount());
        telemetry.addData("Aim status", aim.getStatus());
        telemetry.addData("Turret status", turret.getStatus());
        telemetry.addData("aim.cellPitchDeg", aim.getPitchDegrees());
        telemetry.addData("aim.approachDeg", aim.getApproachDegrees());
        telemetry.addData("aim.rangeInches", aim.getRangeInches());
        telemetry.addData("aim.exposureAgeMs", aim.getExposureAgeMs());
        telemetry.addData("aim.goalDeg", aim.getTurretGoalDegrees());
        telemetry.addData("aim.candidateGoalDeg (unverified)", aim.getCandidateGoalDegrees());
        telemetry.addData("turret.actualDeg", turret.getPositionDegrees());
        telemetry.addData("turret.profileDeg", turret.getProfilePositionDegrees());
        telemetry.addData("turret.power", turret.getOutput());
        telemetry.addData("Azimuth settled (NOT shot ready)", aim.hasAim() && turret.isAtTarget() && !aim.isPredicted());
        telemetry.addLine("GP1 LB + stick X: jog | B: zero | Y: clear fault | RT: hold aim | X: cancel");
        telemetry.addLine("GP2 LB + sticks: drive after field pose is set. Hold RT stops chassis.");
        if (releaseRequired) telemetry.addLine("Release RT/LB, center turret stick, release X/B/Y and GP2 LB to arm.");
    }
    private void shutdown(RuntimeException failure) {
        try { stop(); } catch (RuntimeException stopFailure) { failure.addSuppressed(stopFailure); }
        throw failure;
    }
    @Override public void stop() {
        if (stopped) return;
        stopped = true; active = false;
        try { if (turret != null) turret.stop(); }
        finally {
            try { if (drive != null) drive.stop(); }
            finally {
                try { if (vision != null) vision.stop(); }
                finally { if (aim != null) aim.clear(); Scheduler.reset(); }
            }
        }
    }
}
