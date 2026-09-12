package org.firstinspires.ftc.teamcode.robot;

public enum Alliance {
    RED, BLUE;

    /**
     * Mutable global read at point of use. Set by localizer opmodes, AutoOpMode's
     * init() (alliance chosen via constructor), and the gamepad2 bumpers in teleop. Silently defaults to RED —
     * verifying the DS telemetry shows the right alliance is a pre-match checklist item.
     *
     * KICKOFF TASK: re-add a per-alliance Pose payload (red value + mirror()) if the
     * new game has a key alliance-specific landmark, as DECODE did with its goal.
     */
    public static Alliance current = Alliance.RED;
}
