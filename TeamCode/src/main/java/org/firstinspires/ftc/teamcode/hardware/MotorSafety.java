package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/** Uncached shutdown writes; a failed device must not leave the other motors powered. */
public final class MotorSafety {
    private MotorSafety() { }

    public static void stopAndBrake(DcMotorEx... motors) {
        RuntimeException failure = null;
        for (DcMotorEx motor : motors) {
            if (motor == null) continue; // Includes partially initialized hardware.
            try { motor.setPower(0); }
            catch (RuntimeException fault) { failure = collect(failure, fault); }
            try { motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); }
            catch (RuntimeException fault) { failure = collect(failure, fault); }
        }
        if (failure != null) throw failure;
    }

    private static RuntimeException collect(RuntimeException first, RuntimeException next) {
        if (first == null) return next;
        if (first != next) first.addSuppressed(next);
        return first;
    }
}
