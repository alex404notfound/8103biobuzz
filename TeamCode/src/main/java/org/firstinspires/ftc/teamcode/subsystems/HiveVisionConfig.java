package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

/** Settings for the standalone Hive Vision test. All settings are captured at INIT. */
@Config
public final class HiveVisionConfig {
    private HiveVisionConfig() { }

    public static String hardwareName = "limelight";
    /** Put the neural detector in its own slot; the existing AprilTag default is slot 0. */
    public static int pipelineIndex = 1;
    public static int pollRateHz = 100;
    public static double minimumConfidence = 0.44;
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
}
