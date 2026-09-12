package org.firstinspires.ftc.teamcode.pedroPathing.procedures;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.TuningSafety;

import java.util.Map;

/**
 * AutoTune starts each LinearOpMode automatically. This supplies a stopped setup/arming stage,
 * checked hardware boundaries and finally cleanup that every procedure must use.
 */
abstract class SafeTuningOpMode<Result> extends TuningOpMode<Result> {
    private final DcMotorEx[] motors = new DcMotorEx[4];
    private Mecanum rawDrivetrain;
    private Localizer rawLocalizer;
    private GoBildaPinpointDriver pinpoint;
    private TuningSafety safety;
    private Follower follower;
    private boolean restoreOffsets;
    private double originalXOffset;
    private double originalYOffset;
    private String localizationStatus = "WAITING FOR SAMPLE";

    protected SafeTuningOpMode(String name, String description, boolean canStop) {
        super(name, description + "\nKeep clear of the robot. On the gamepad, X recalibrates "
                + "while still; release then press A at READY to arm. B or STOP aborts.", true);
    }

    protected long timeoutNanos() { return 15_000_000_000L; }

    @Override protected final Result runTuningOpMode() throws InterruptedException {
        try {
            String[] names = {
                    Constants.drivetrainConfig.frontLeftName.get(),
                    Constants.drivetrainConfig.frontRightName.get(),
                    Constants.drivetrainConfig.backLeftName.get(),
                    Constants.drivetrainConfig.backRightName.get()
            };
            for (int i = 0; i < names.length; i++) {
                motors[i] = hardwareMap.get(DcMotorEx.class, names[i]);
                motors[i].setPower(0);
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            rawDrivetrain = Constants.createDrivetrain(hardwareMap);
            stopDrive(); // Mecanum's constructor selects FLOAT.
            rawLocalizer = Constants.createGuardedLocalizer(hardwareMap);
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
            safety = new TuningSafety(this::sample, this::stopDrive, this::restoreOffsets,
                    this::isStopRequested, () -> gamepad1.b, System::nanoTime, timeoutNanos());
            waitForStart();
            awaitArming();
            Result result = runSafely();
            // A loop that exits because STOP changed must never report a successful calibration.
            safety.checkpoint();
            return result;
        } finally {
            if (safety != null) safety.close();
            else stopDrive(); // Includes partially constructed hardware.
        }
    }

    protected abstract Result runSafely() throws InterruptedException;

    private void awaitArming() throws InterruptedException {
        boolean previousA = gamepad1.a;
        boolean previousX = false;
        while (true) {
            boolean x = gamepad1.x;
            if (x && !previousX) safety.recalibrate(pinpoint::recalibrateIMU);
            previousX = x;
            boolean ready = safety.prepare();
            boolean a = gamepad1.a;
            telemetry.addData("Pinpoint", ready ? "READY"
                    : "READY".equals(localizationStatus) ? "CALIBRATING - keep still" : localizationStatus);
            telemetry.addLine("Keep still. X recalibrates. Release then press A at READY to arm. B/STOP aborts.");
            telemetry.update();
            if (ready && a && !previousA) {
                safety.begin();
                return;
            }
            previousA = a;
            Thread.sleep(10);
        }
    }

    private boolean sample() {
        try {
            rawLocalizer.update();
            boolean finite = Constants.isFinitePose(rawLocalizer.pose())
                    && Constants.isFiniteVelocity(rawLocalizer.velocity());
            localizationStatus = finite ? "READY" : "NONFINITE SAMPLE";
            return finite;
        } catch (Constants.LocalizationNotReady fault) {
            localizationStatus = fault.getMessage();
            return false;
        }
    }

    protected final void checkpoint() { safety.checkpoint(); }

    protected final Localizer localizer() {
        return new Localizer() {
            @Override public void setPose(Pose pose) {
                safety.command(() -> {
                    if (!Constants.isFinitePose(pose)) throw new IllegalArgumentException("Pose must be finite");
                    rawLocalizer.setPose(pose);
                });
            }
            @Override public MotionState state() { return rawLocalizer.state(); }
            @Override public void update() { safety.checkpoint(); }
            @Override public void reset() {
                throw new IllegalStateException("Use X before arming to recalibrate Pinpoint");
            }
        };
    }

    protected final Drivetrain drivetrain() {
        return new Drivetrain() {
            @Override public void drive(DrivePowers powers, boolean manual) {
                safety.command(() -> {
                    if (powers == null || !Double.isFinite(powers.forward())
                            || !Double.isFinite(powers.strafe()) || !Double.isFinite(powers.turn()))
                        throw new IllegalArgumentException("Tuning produced nonfinite motor power");
                    rawDrivetrain.drive(powers, manual);
                });
            }
            @Override public double maxScaling(DrivePowers current, DrivePowers delta) {
                return rawDrivetrain.maxScaling(current, delta);
            }
            @Override public void stop() { rawDrivetrain.stop(true); }
            @Override public void stop(boolean brake) { rawDrivetrain.stop(brake); }
            @Override public Map<String, Object> debug() { return rawDrivetrain.debug(); }
            @Override public double interpolateVelocity(double x, double y, double theta) {
                return rawDrivetrain.interpolateVelocity(x, y, theta);
            }
        };
    }

    protected final Follower follower() {
        if (!Constants.foresightTuned)
            throw new TuningSafety.Aborted("Measure and enter Foresight calibration, then set foresightTuned before path tests");
        if (follower == null)
            follower = new Follower(localizer(), drivetrain(), new Foresight(Constants.foresightConfig));
        return follower;
    }

    /** One raw motor is permitted only for wheel identification, through the same fresh-sample gate. */
    protected final void spinMotor(int index, double power) {
        safety.command(() -> {
            if (index < 0 || index >= motors.length || !Double.isFinite(power) || Math.abs(power) > 0.3)
                throw new IllegalArgumentException("Wheel identification is limited to 30% power");
            motors[index].setPower(power);
        });
    }

    protected final void zeroOffsetsTemporarily() {
        checkpoint();
        originalXOffset = pinpoint.getXOffset(DistanceUnit.INCH);
        originalYOffset = pinpoint.getYOffset(DistanceUnit.INCH);
        if (!Double.isFinite(originalXOffset) || !Double.isFinite(originalYOffset))
            throw new IllegalArgumentException("Cannot preserve nonfinite Pinpoint offsets");
        // Register restoration before the first write, including a partially failed I2C write.
        restoreOffsets = true;
        pinpoint.setOffsets(0, 0, DistanceUnit.INCH);
    }

    private void restoreOffsets() {
        if (!restoreOffsets) return;
        restoreOffsets = false;
        pinpoint.setOffsets(originalXOffset, originalYOffset, DistanceUnit.INCH);
    }

    private void stopDrive() {
        if (follower != null) follower.stop();
        try {
            if (rawDrivetrain != null) rawDrivetrain.stop(true);
        } finally {
            // Force SDK writes: Pedro's cached power threshold must not suppress a small final zero.
            RuntimeException failure = null;
            for (DcMotorEx motor : motors) {
                if (motor == null) continue;
                try {
                    try { motor.setPower(0); }
                    finally { motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); }
                } catch (RuntimeException fault) {
                    if (failure == null) failure = fault;
                    else failure.addSuppressed(fault);
                }
            }
            if (failure != null) throw failure;
        }
    }
}
