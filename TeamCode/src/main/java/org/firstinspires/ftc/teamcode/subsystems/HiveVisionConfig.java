package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Settings for the standalone Hive Vision test. All settings are captured at INIT. */
@Configurable
public final class HiveVisionConfig {
    private HiveVisionConfig() { }

    public static String hardwareName = "limelight";
    /** Put the neural detector in its own slot; the existing AprilTag default is slot 0. */
    public static int pipelineIndex = 1;
    public static int pollRateHz = 100;
    /** Hive Vision v2 release recommends 0.80-0.90; tune against actual field lighting. */
    public static double minimumConfidence = 0.80;
    /** Set only after checking the raw confidence readout for this camera firmware. */
    public static boolean confidenceIsPercent = false;
    /** Includes capture/processing latency and time spent receiving/reusing a frame. */
    public static double maximumFrameAgeMs = 150;
    public static int confirmationFrames = 4;
    public static double associationGateDegrees = 12;

    /** Angles/classes work before calibration. Position estimates require measured values. */
    public static boolean geometryVerified = false;
    public static double cameraHeightInches = 0;
    public static double ballCenterHeightInches = 0;
    /** Down-positive pitch; yaw is counterclockwise/left-positive. Assumes zero camera roll. */
    public static double cameraPitchDownDegrees = 0;
    public static double cameraYawLeftDegrees = 0;
    public static double cameraForwardInches = 0;
    public static double cameraLeftInches = 0;
    public static double maximumCameraRangeInches = 120;

    /** One detector-camera setup shared by stationary and moving diagnostics. Caller owns stop(). */
    public static Limelight3A startCamera(HardwareMap hardwareMap, int pipeline) {
        int rate = pollRateHz;
        if (pipeline < 0 || pipeline > 9 || rate < 1 || rate > 250)
            throw new IllegalArgumentException("Hive Vision pipeline must be 0..9 and pollRateHz must be 1..250");
        Limelight3A camera = hardwareMap.get(Limelight3A.class, hardwareName);
        try {
            camera.setPollRateHz(rate);
            if (!camera.pipelineSwitch(pipeline))
                throw new IllegalStateException("Could not select Hive Vision pipeline " + pipeline);
            camera.start();
            return camera;
        } catch (RuntimeException failure) {
            try { camera.stop(); }
            catch (RuntimeException stopFailure) { failure.addSuppressed(stopFailure); }
            throw failure;
        }
    }
}
