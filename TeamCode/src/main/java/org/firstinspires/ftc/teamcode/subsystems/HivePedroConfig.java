package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

/** Practice-only approach settings, captured at INIT. Distances are from robot center to ball center. */
@Config
public final class HivePedroConfig {
    private HivePedroConfig() { }

    public static double approachDistanceInches = 30;
    public static double stopDistanceInches = 18;
    public static double distanceToleranceInches = 1;
    public static double angleToleranceDegrees = 4;
    public static double forwardGain = 0.025;
    public static double turnGain = 0.012;
    public static double maximumMotorPower = 0.25;
    public static double maximumPathSpeedInchesPerSecond = 10;
    public static double maximumPathAcceleration = 10;
    public static double maximumAttemptSeconds = 8;
    public static double maximumTravelInches = 48;
    public static double maximumTargetShiftInches = 8;
}
