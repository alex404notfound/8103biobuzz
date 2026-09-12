package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        // X pod measures forward travel; its offset is measured along robot Y.
        // Setup values only: replace with the Pinpoint AutoTune measurements.
        c.xPodOffset.set(0.0);
        c.yPodOffset.set(0.0);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        // INIT X explicitly recalibrates while stationary; constructing a follower preserves pose.
        c.resetMode.set(PinpointLocalizer.ResetMode.NONE);
    });

    /** Set true only after replacing foresightConfig with this robot's AutoTune output. */
    public static boolean foresightTuned = false;

    /**
     * Unmeasured scaffold for constructing the follower during TeleOp and sensor checks.
     * These are NOT measured gains. Pedro 2 predictive-braking values cannot fill the v3 model.
     * The path gates provide the protection; these numbers alone do not prevent motor output.
     * Run AutoTune Foresight, replace this entire block, then set foresightTuned=true.
     */
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        c.headingFeedback.set(Controller.zero);
        c.forwardTranslational.set(Controller.zero);
        c.strafeTranslational.set(Controller.zero);
        c.brake.set(Controller.zero);
        c.coast.set(Controller.zero);
        c.linearBrakeCoefficients.set(Matrix.diag(0, 0));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0, 0));
        c.headingBrakeCoefficients.set(Vector2D.zero());
        c.maxAchievableForwardVelocity.set(1.0);
        c.maxAchievableStrafeVelocity.set(1.0);
        c.naturalForwardDeceleration.set(1.0);
        c.naturalStrafeDeceleration.set(1.0);
    });

    public static Mecanum createDrivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, drivetrainConfig) {
            @Override public void drive(DrivePowers powers, boolean manual) {
                // CachedMotor ignores nonfinite values, which would retain the previous power.
                // Stop the complete drivetrain before allowing that partial update to occur.
                if (powers == null || !Double.isFinite(powers.forward())
                        || !Double.isFinite(powers.strafe()) || !Double.isFinite(powers.turn())) {
                    super.stop();
                    throw new LocalizationNotReady("NON_FINITE_DRIVE_COMMAND");
                }
                super.drive(powers, manual);
            }
        };
    }

    public static Localizer createGuardedLocalizer(HardwareMap hardwareMap) {
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, localizerConfig);
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, localizerConfig.name.get());
        // Wrap after construction: PinpointLocalizer reads a sample in its constructor, when the
        // device can legitimately still be calibrating. Motor-control updates must pass this guard.
        return new Localizer() {
            @Override public void update() {
                localizer.update();
                GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
                if (status != GoBildaPinpointDriver.DeviceStatus.READY)
                    throw new LocalizationNotReady(String.valueOf(status));
                if (!isFinitePose(pose()) || !isFiniteVelocity(velocity()))
                    throw new LocalizationNotReady("NON_FINITE_POSE_OR_VELOCITY");
            }
            @Override public MotionState state() { return localizer.state(); }
            @Override public void setPose(Pose pose) { localizer.setPose(pose); }
            @Override public void reset() { localizer.reset(); }
        };
    }

    public static Follower createGuardedFollower(HardwareMap hardwareMap) {
        return new Follower(createGuardedLocalizer(hardwareMap), createDrivetrain(hardwareMap),
                new Foresight(foresightConfig));
    }

    public static boolean isFinitePose(Pose pose) {
        return pose != null && Double.isFinite(pose.x()) && Double.isFinite(pose.y())
                && Double.isFinite(pose.heading());
    }

    public static boolean isFiniteVelocity(Velocity velocity) {
        return velocity != null && Double.isFinite(velocity.vx) && Double.isFinite(velocity.vy)
                && Double.isFinite(velocity.omega);
    }

    /** Caught by Drivetrain, never interpreted as natural Ivy path completion. */
    public static final class LocalizationNotReady extends RuntimeException {
        public LocalizationNotReady(String status) { super(status); }
    }
}
