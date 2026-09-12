package org.firstinspires.ftc.teamcode.pedroPathing.procedures;

import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.utils.Angle;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.OffsetCalibration;

/** Manual measurements keep motors stopped and never persist temporary zero-offset compensation. */
public final class PinpointTuner extends Procedure {
    public PinpointTuner() {
        super("Pinpoint setup and offsets", "Check pod model, distance scale and directions, then measure offsets with a manual half-turn.");
    }

    @Override public void run() throws InterruptedException {
        confirmation("Check the odometry hardware", "Verify Constants.localizerConfig.name, podType (or custom ticksPerUnit), "
                + "and distance units match the installed hardware. Current device: " + Constants.localizerConfig.name.get()
                + ". These are unmeasured starting settings. All measurements in this procedure keep the motors stopped.");
        Inputs input = inputs("Push distance", "Mark a straight distance in inches. Push without rotating, first forward and then left.");
        Inputs.Field<Double> distance = input.d("Measured push distance (inches)").withDefault(24.0);
        awaitInputs(input);
        CalibrationValues.range(distance.get(), 6, 96, "Push distance");
        double forward = runOpMode(new PinpointPush(true, distance.get()));
        double left = runOpMode(new PinpointPush(false, distance.get()));
        GoBildaPinpointDriver.EncoderDirection xDirection = corrected(Constants.localizerConfig.xPodDirection.get(), forward);
        GoBildaPinpointDriver.EncoderDirection yDirection = corrected(Constants.localizerConfig.yPodDirection.get(), left);
        result("Forward reported inches", forward);
        result("Left reported inches", left);
        result("xPodDirection", xDirection);
        result("yPodDirection", yDirection);
        String directions = "// Apply inside Constants.localizerConfig.\n"
                + "c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection." + xDirection + ");\n"
                + "c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection." + yDirection + ");\n";
        if (xDirection != Constants.localizerConfig.xPodDirection.get()
                || yDirection != Constants.localizerConfig.yPodDirection.get()
                || Math.abs(Math.abs(forward) / distance.get() - 1) > 0.10
                || Math.abs(Math.abs(left) / distance.get() - 1) > 0.10) {
            result("Next step", "Apply the reported directions and correct pod resolution if the distance error exceeds 10%. Rerun before measuring offsets.");
            code(Language.JAVA, directions);
            return;
        }
        OffsetCalibration.Result offsets = runOpMode(new PinpointOffsets());
        // Pedro 3 xPodOffset is the forward pod's lateral offset; yPodOffset is the strafe pod's forward offset.
        result("xPodOffset (inches)", offsets.forwardPodY);
        result("yPodOffset (inches)", offsets.strafePodX);
        code(Language.JAVA, directions
                + "c.xPodOffset.set(" + offsets.forwardPodY + ");\n"
                + "c.yPodOffset.set(" + offsets.strafePodX + ");\n"
                + "c.offsetUnits.set(DistanceUnit.INCH);\n"
                + "// Saved device offsets were restored. Copy these measurements into Constants and verify again.\n");
    }

    private static GoBildaPinpointDriver.EncoderDirection corrected(GoBildaPinpointDriver.EncoderDirection configured, double displacement) {
        if (displacement > 0) return configured;
        return configured == GoBildaPinpointDriver.EncoderDirection.FORWARD
                ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }
}

final class PinpointPush extends SafeTuningOpMode<Double> {
    private final boolean forward;
    PinpointPush(boolean forward, double distance) {
        super(forward ? "Forward pod check" : "Left pod check",
                "After arming, push the robot " + distance + " inches " + (forward ? "forward" : "left")
                        + " without turning, then release and press A to accept. STOP aborts.", true);
        this.forward = forward;
    }
    @Override protected long timeoutNanos() { return 120_000_000_000L; }
    @Override protected Double runSafely() throws InterruptedException {
        Localizer localizer = localizer();
        localizer.setPose(Pose.zero());
        boolean released = false;
        while (true) {
            localizer.update();
            double displacement = forward ? localizer.pose().x() : localizer.pose().y();
            telemetry.addData("Reported inches", displacement);
            telemetry.addLine("Finish the measured push, then release/press A. B/STOP aborts.");
            telemetry.update();
            if (!gamepad1.a) released = true;
            if (released && gamepad1.a) {
                if (Math.abs(displacement) < 3) throw new IllegalArgumentException("Too little pod movement; check wiring and repeat");
                if (Math.abs(Angle.normalizeSigned(localizer.pose().heading())) > Math.toRadians(10))
                    throw new IllegalArgumentException("Robot rotated during the push; repeat without turning");
                return displacement;
            }
            Thread.sleep(10);
        }
    }
}

final class PinpointOffsets extends SafeTuningOpMode<OffsetCalibration.Result> {
    PinpointOffsets() {
        super("Pinpoint offset measurement", "After arming, rotate the robot manually in place about 180 degrees in either direction. "
                + "Release and press A once the robot is still. Temporary zero offsets are restored on every exit.", true);
    }
    @Override protected long timeoutNanos() { return 120_000_000_000L; }
    @Override protected OffsetCalibration.Result runSafely() throws InterruptedException {
        Localizer localizer = localizer();
        zeroOffsetsTemporarily();
        localizer.setPose(Pose.zero());
        localizer.update();
        double previousHeading = localizer.pose().heading();
        double totalHeading = 0;
        boolean released = false;
        while (true) {
            localizer.update();
            Pose pose = localizer.pose();
            totalHeading += Angle.normalizeSigned(pose.heading() - previousHeading);
            previousHeading = pose.heading();
            telemetry.addData("Turn (degrees)", Math.toDegrees(totalHeading));
            telemetry.addLine("Turn about 180 degrees; stop moving and release/press A. B/STOP aborts.");
            telemetry.update();
            if (!gamepad1.a) released = true;
            if (released && gamepad1.a) {
                if (!OffsetCalibration.hasCompletedHalfTurn(totalHeading) || Math.abs(totalHeading) > Math.toRadians(240))
                    throw new IllegalArgumentException("Use a half-turn between 180 and 240 degrees; repeat the measurement");
                if (localizer.velocity().toVector2D().magnitude() > 0.5 || Math.abs(localizer.velocity().omega) > 0.05)
                    throw new IllegalArgumentException("Robot must be still before accepting the offset measurement");
                return OffsetCalibration.calculate(pose.x(), pose.y(), totalHeading);
            }
            Thread.sleep(10);
        }
    }
}
