package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathBuilder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import java.util.function.LongSupplier;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Config
public class Drivetrain {
    public static PIDFCoefficients headingCoefficients = new PIDFCoefficients(1.75, 0, 0.09, 0);
    private static Pose poseTransfer = new Pose();
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private final Follower follower;
    private final Telemetry telemetry;
    private final GoBildaPinpointDriver pinpoint;
    private final LongSupplier clock;
    private boolean localizationSampleSeen;
    private boolean followingControlEnabled;
    private long lastLocalizationSample;
    private boolean calibrationPending;
    private long calibrationStarted;
    private String localizationStatus = "WAITING_FOR_SAMPLE";
    private static final long MAX_SAMPLE_AGE_NANOS = 250_000_000L;
    private static final long CALIBRATION_MIN_NANOS = 300_000_000L;
    private final PIDFController headingController = new PIDFController(headingCoefficients);
    private boolean lockHeading = false;
    private double headingTargetRadians = 0;

    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry) {
        this(Constants.createGuardedFollower(hardwareMap), hardwareMap, telemetry);
    }

    Drivetrain(Follower follower, HardwareMap hardwareMap, Telemetry telemetry) {
        this(follower, hardwareMap, telemetry, System::nanoTime);
    }

    Drivetrain(Follower follower, HardwareMap hardwareMap, Telemetry telemetry, LongSupplier clock) {
        this.follower = follower;
        this.clock = clock;
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConstants.hardwareMapName);
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.telemetry = telemetry;
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
     * Blue callers mirror BEFORE calling: lockHeading(180 - redDegrees).
     */
    public void lockHeading(double targetDegrees) {
        lockHeading = true;
        headingTargetRadians = Math.toRadians(targetDegrees);
    }

    public void unlockHeading() {
        lockHeading = false;
    }

    public void arcadeDrive(double forward, double strafe, double turn, Alliance alliance) {
        if (!isLocalizationReady()) { stop(); return; }
        double headingRadians = follower.getHeading();

        forward = signedSquare(forward);
        strafe = signedSquare(strafe);

        if (lockHeading) {
            headingController.updateError(AngleUnit.normalizeRadians(headingTargetRadians - headingRadians));
            turn = -headingController.run();
        } else {
            turn = signedSquare(turn);
        }

        if (alliance == Alliance.BLUE) headingRadians += Math.PI;

        double x = strafe * Math.cos(headingRadians) + forward * Math.sin(headingRadians);
        double y = strafe * -Math.sin(headingRadians) + forward * Math.cos(headingRadians);
        y *= 1.1;

        double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(turn), 1);

        frontLeft.setPower((y + x + turn) / denominator);
        frontRight.setPower((y - x - turn) / denominator);
        backLeft.setPower((y - x + turn) / denominator);
        backRight.setPower((y + x - turn) / denominator);
    }

    public Pose getPose() {
        return follower.getPose();
    }

    /** Robot velocity in field inches/second and radians/second. */
    public Pose getVelocity() { return follower.getPoseTracker().getLocalizer().getVelocity(); }

    public double getHeading() {
        return follower.getHeading();
    }

    public PathBuilder pathBuilder() {
        return follower.pathBuilder();
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public void setStartingPose(Pose pose) {
        follower.setStartingPose(pose);
    }

    public void usePreviousStartingPose() {
        setStartingPose(poseTransfer);
    }

    public Command followPath(PathChain path) {
        // Ivy's Pedro factory starts/tests the follower, but supplies no ownership or cleanup.
        return follow(follower, path)
                .requiring(this)
                .setStart(() -> {
                    followingControlEnabled = true;
                    follower.followPath(path, follower.getMaxPowerScaling(), follower.constants.automaticHoldEnd);
                })
                .setEnd(condition -> {
                    if (condition != EndCondition.NATURALLY) stopFollowing();
                });
    }

    public void periodic() {
        try {
            // During an explicit calibration, sample only; never run drive controllers.
            if (calibrationPending || !followingControlEnabled) {
                follower.updatePose();
                follower.updateDrivetrain(); // refresh configured motor directions without drive output
            }
            else follower.update();
            localizationStatus = String.valueOf(pinpoint.getDeviceStatus());
            if (!Constants.isFinitePose(follower.getPose())) localizationStatus = "NON_FINITE_POSE";
            if (calibrationPending && clock.getAsLong() - calibrationStarted >= CALIBRATION_MIN_NANOS
                    && "READY".equals(localizationStatus)) calibrationPending = false;
        } catch (Constants.LocalizationNotReady fault) {
            localizationStatus = fault.getMessage();
        }
        localizationSampleSeen = true;
        lastLocalizationSample = clock.getAsLong();
        if (isLocalizationReady()) poseTransfer = follower.getPose();
        else stop();

        telemetry.addData("Pinpoint Status", getLocalizationStatus());

        telemetry.addData("Drivetrain X", follower.getPose().getX());
        telemetry.addData("Drivetrain Y", follower.getPose().getY());
        telemetry.addData("Drivetrain Heading", Math.toDegrees(follower.getHeading()));
        telemetry.addData("Drivetrain Heading Locked", lockHeading);
        telemetry.addData("Drivetrain Heading Target", Math.toDegrees(headingTargetRadians));
    }

    /** Readiness always refers to a recent completed localization update. */
    public boolean isLocalizationReady() {
        return localizationSampleSeen && !calibrationPending && "READY".equals(localizationStatus)
                && clock.getAsLong() - lastLocalizationSample <= MAX_SAMPLE_AGE_NANOS
                && Constants.isFinitePose(follower.getPose());
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
        follower.breakFollowing();
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
