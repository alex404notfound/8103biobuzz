package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.field.Canvas;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.pedropathing.controllers.PIDController;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import dev.nextftc.robot.Mechanism;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import java.util.function.LongSupplier;
import java.util.function.Consumer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.hardware.MotorSafety;
import org.firstinspires.ftc.teamcode.pedroPathing.GuardedMecanum;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.telemetry.FieldMapTelemetry;
import org.firstinspires.ftc.teamcode.telemetry.PanelsFieldTelemetry;

@Configurable
public class Drivetrain implements Mechanism {
    // Optional TeleOp heading hold gains; tune on the actual robot in Panels.
    public static double headingP = 0, headingI = 0, headingD = 0;
    @IgnoreConfigurable private static Pose poseTransfer = Pose.zero();
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private final Follower follower;
    private final Telemetry telemetry;
    private final GoBildaPinpointDriver pinpoint;
    private final LongSupplier clock;
    private final FieldMapTelemetry fieldMap;
    private boolean localizationSampleSeen;
    private boolean followingControlEnabled;
    private long lastLocalizationSample;
    private boolean calibrationPending;
    private long calibrationStarted;
    private String localizationStatus = "WAITING_FOR_SAMPLE";
    private static final long MAX_SAMPLE_AGE_NANOS = 250_000_000L;
    private static final long CALIBRATION_MIN_NANOS = 300_000_000L;
    private final PIDController headingController = new PIDController(headingP, headingI, headingD);
    private boolean lockHeading = false;
    private double headingTargetRadians = 0;

    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry) {
        this(Constants.createGuardedFollower(hardwareMap), hardwareMap, telemetry,
                System::nanoTime, PanelsFieldTelemetry::send);
    }

    Drivetrain(Follower follower, HardwareMap hardwareMap, Telemetry telemetry) {
        this(follower, hardwareMap, telemetry, System::nanoTime);
    }

    Drivetrain(Follower follower, HardwareMap hardwareMap, Telemetry telemetry, LongSupplier clock) {
        this(follower, hardwareMap, telemetry, clock, frame -> { });
    }

    Drivetrain(Follower follower, HardwareMap hardwareMap, Telemetry telemetry, LongSupplier clock,
               Consumer<Canvas> fieldFrames) {
        this.follower = follower;
        this.clock = clock;
        this.fieldMap = new FieldMapTelemetry(telemetry, fieldFrames, clock);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
        frontLeft = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.frontLeftName.get());
        frontRight = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.frontRightName.get());
        backLeft = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.backLeftName.get());
        backRight = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.backRightName.get());

        this.telemetry = telemetry;
        stop();
    }

    private static double signedSquare(double raw) {
        return Math.signum(raw) * Math.pow(raw, 2);
    }

    public static void localize(Pose pose) {
        poseTransfer = pose;
    }

    /**
     * Hold an absolute field heading (degrees, red frame) with the heading PIDF until
     * the driver turns (see unlockHeading / the teleop turn-stick binding).
     * Blue callers rotate BEFORE calling: lockHeading(redDegrees + 180).
     */
    public void lockHeading(double targetDegrees) {
        headingController.reset();
        lockHeading = true;
        headingTargetRadians = Math.toRadians(targetDegrees);
    }

    public void unlockHeading() {
        lockHeading = false;
        headingController.reset();
    }

    public void arcadeDrive(double forward, double strafe, double turn, Alliance alliance) {
        if (!isLocalizationReady()) { stop(); return; }
        double headingRadians = follower.pose().heading();

        forward = signedSquare(forward);
        strafe = signedSquare(strafe);

        if (lockHeading) {
            headingController.kP = headingP;
            headingController.kI = headingI;
            headingController.kD = headingD;
            turn = -headingController.calculate(0,
                    AngleUnit.normalizeRadians(headingTargetRadians - headingRadians));
        } else {
            turn = signedSquare(turn);
        }

        if (alliance == Alliance.BLUE) headingRadians += Math.PI;

        double x = strafe * Math.cos(headingRadians) + forward * Math.sin(headingRadians);
        double y = strafe * -Math.sin(headingRadians) + forward * Math.cos(headingRadians);
        y *= 1.1;

        double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(turn), 1);

        // Use Pedro's motor owner for manual output too, so its cached powers remain correct.
        follower.drivetrain.drive(new DrivePowers(y / denominator, -x / denominator,
                -turn / denominator), true);
    }

    public Pose getPose() {
        return follower.pose();
    }

    public void setFieldOverlay(Consumer<Canvas> overlay) { fieldMap.setAdditionalOverlay(overlay); }

    /** Linear robot-relative output through Pedro. Forward/left/CCW are positive; caller owns this mechanism. */
    public void driveRobotRelative(double forward, double left, double counterclockwise) {
        if (!Double.isFinite(forward) || !Double.isFinite(left) || !Double.isFinite(counterclockwise)
                || !isLocalizationReady()) { stop(); return; }
        if (followingControlEnabled) stopFollowing();
        unlockHeading();
        double scale = Math.max(1, Math.abs(forward) + Math.abs(left) + Math.abs(counterclockwise));
        follower.drivetrain.drive(new DrivePowers(forward / scale, left / scale, counterclockwise / scale), true);
    }

    /** Instance-local cap applies to both Pedro paths and direct camera corrections. */
    public double getMotorPowerLimit() { return guardedMotorOwner().getPowerLimit(); }
    public void setMotorPowerLimit(double limit) { guardedMotorOwner().setPowerLimit(limit); }

    private GuardedMecanum guardedMotorOwner() {
        if (!(follower.drivetrain instanceof GuardedMecanum))
            throw new IllegalStateException("A power-limited test requires the configured guarded Pedro drivetrain");
        return (GuardedMecanum) follower.drivetrain;
    }

    /** Robot velocity in field inches/second and radians/second. */
    public Velocity getVelocity() { return follower.velocity(); }

    public double getHeading() {
        return follower.pose().heading();
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
        fieldMap.resetTrail();
    }

    public void setStartingPose(Pose pose) {
        setPose(pose);
    }

    public void usePreviousStartingPose() {
        setStartingPose(poseTransfer);
    }

    public boolean isPathFollowingConfigured() { return Constants.foresightTuned; }

    public Command followPath(Path path) {
        // Own the v3 lifecycle directly: the stock Ivy adapter uses parametric completion
        // and supplies neither mechanism ownership nor cancellation cleanup.
        return Command.build()
                .requiring(this)
                .setStart(() -> {
                    if (!isPathFollowingConfigured()) {
                        stop();
                        throw new IllegalStateException("Run Pedro AutoTune and configure Foresight before following paths");
                    }
                    followingControlEnabled = true;
                    follower.follow(path);
                })
                // Ivy 1.1's default completes at the parametric endpoint. A sequence should wait
                // for v3's end hold to settle (or IDLE when holdEnd is explicitly disabled).
                .setDone(() -> follower.idle() || (follower.holding() && !follower.isBusy()))
                .setEnd(condition -> {
                    if (condition != EndCondition.NATURALLY) stopFollowing();
                });
    }

    public void periodic() {
        try {
            // During an explicit calibration, sample only; never run drive controllers.
            if (calibrationPending || !followingControlEnabled) {
                follower.localizer.update();
            }
            else follower.update();
            localizationStatus = String.valueOf(pinpoint.getDeviceStatus());
            if (!Constants.isFinitePose(follower.pose()) || !Constants.isFiniteVelocity(follower.velocity()))
                localizationStatus = "NON_FINITE_POSE_OR_VELOCITY";
            if (calibrationPending && clock.getAsLong() - calibrationStarted >= CALIBRATION_MIN_NANOS
                    && "READY".equals(localizationStatus)) calibrationPending = false;
        } catch (Constants.LocalizationNotReady fault) {
            localizationStatus = fault.getMessage();
        }
        localizationSampleSeen = true;
        lastLocalizationSample = clock.getAsLong();
        if (isLocalizationReady()) poseTransfer = follower.pose();
        else stop();

        telemetry.addData("Pinpoint Status", getLocalizationStatus());

        telemetry.addData("Drivetrain X", follower.pose().x());
        telemetry.addData("Drivetrain Y", follower.pose().y());
        telemetry.addData("Drivetrain Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Drivetrain Heading Locked", lockHeading);
        telemetry.addData("Drivetrain Heading Target", Math.toDegrees(headingTargetRadians));
        fieldMap.update(follower.pose(), isLocalizationReady(), getLocalizationStatus());
    }

    /** Readiness always refers to a recent completed localization update. */
    public boolean isLocalizationReady() {
        return localizationSampleSeen && !calibrationPending && "READY".equals(localizationStatus)
                && clock.getAsLong() - lastLocalizationSample <= MAX_SAMPLE_AGE_NANOS
                && Constants.isFinitePose(follower.pose()) && Constants.isFiniteVelocity(follower.velocity());
    }

    public String getLocalizationStatus() {
        if (calibrationPending) return "CALIBRATING - keep robot still (" + localizationStatus + ")";
        if (localizationSampleSeen && clock.getAsLong() - lastLocalizationSample > MAX_SAMPLE_AGE_NANOS)
            return "STALE_LOCALIZATION_SAMPLE";
        return localizationStatus;
    }

    /** Operator-triggered stationary INIT calibration; preserves the current field pose. */
    public void recalibrateLocalization() {
        stop();
        calibrationPending = true;
        calibrationStarted = clock.getAsLong();
        localizationSampleSeen = false;
        pinpoint.recalibrateIMU();
    }

    /** Cancels Pedro's active path or end hold without changing localization. */
    public void stopFollowing() {
        followingControlEnabled = false;
        try {
            try { follower.stop(); }
            finally { follower.drivetrain.stop(); }
        } finally {
            // Pedro changes mode before output, and its cached writes can skip small zeros.
            MotorSafety.stopAndBrake(frontLeft, frontRight, backLeft, backRight);
        }
    }

    /** Vision corrects translation; MegaTag2 yaw is not an independent heading measurement. */
    public void applyVisionTranslation(Pose candidate) {
        if (!isLocalizationReady() || !Constants.isFinitePose(candidate)) return;
        Pose corrected = candidate.withHeading(getHeading());
        stopFollowing();
        setPose(corrected);
    }

    public void stop() {
        lockHeading = false;
        stopFollowing();
    }
}
