package org.firstinspires.ftc.teamcode.pedroPathing.procedures;

import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/** Checks the configured hardware one wheel at a time before any floor-driving calibration. */
public final class MecanumTuner extends Procedure {
    public MecanumTuner() {
        super("Mecanum setup", "Identify each configured motor and its direction with the robot securely raised.");
    }

    @Override public void run() throws InterruptedException {
        String[] labels = {"front left", "front right", "back left", "back right"};
        String[] names = {
                Constants.drivetrainConfig.frontLeftName.get(), Constants.drivetrainConfig.frontRightName.get(),
                Constants.drivetrainConfig.backLeftName.get(), Constants.drivetrainConfig.backRightName.get()
        };
        DcMotorSimple.Direction[] directions = {
                Constants.drivetrainConfig.frontLeftDirection.get(), Constants.drivetrainConfig.frontRightDirection.get(),
                Constants.drivetrainConfig.backLeftDirection.get(), Constants.drivetrainConfig.backRightDirection.get()
        };
        confirmation("Raise the robot", "Securely support the robot with all four wheels clear of the floor. "
                + "The configured names and directions are unverified starting values. Each wheel runs for 3 seconds at 25% power.");
        for (int i = 0; i < labels.length; i++) {
            runOpMode(new WheelIdentification(i, labels[i], names[i]));
            Inputs observations = inputs(labels[i], "For a forward-moving robot, the bottom of each tire moves toward the back of the robot.");
            Inputs.Field<Boolean> correctMotor = observations.b("Did only the " + labels[i] + " wheel spin?").withDefault(false);
            Inputs.Field<Boolean> forward = observations.b("Did that wheel turn in the forward-driving direction?").withDefault(false);
            awaitInputs(observations);
            if (!correctMotor.get()) {
                abort("Fix the HardwareMap name/port for " + labels[i] + " (currently " + names[i] + ") in Constants, then rerun.");
                return;
            }
            if (!forward.get()) directions[i] = directions[i] == DcMotorSimple.Direction.FORWARD
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
        }
        String[] fields = {"frontLeft", "frontRight", "backLeft", "backRight"};
        StringBuilder code = new StringBuilder("// Apply inside Constants.drivetrainConfig, then rerun to verify.\n");
        for (int i = 0; i < fields.length; i++) {
            result(labels[i], names[i] + " / " + directions[i]);
            code.append("c.").append(fields[i]).append("Direction.set(DcMotorSimple.Direction.")
                    .append(directions[i]).append(");\n");
        }
        code(Language.JAVA, code.toString());
    }
}

final class WheelIdentification extends SafeTuningOpMode<Void> {
    private final int motor;
    WheelIdentification(int motor, String label, String hardwareName) {
        super("Identify " + label, "The " + label + " wheel (" + hardwareName + ") will spin for 3 seconds.", true);
        this.motor = motor;
    }
    @Override protected Void runSafely() throws InterruptedException {
        long end = System.nanoTime() + 3_000_000_000L;
        while (System.nanoTime() < end) {
            spinMotor(motor, 0.25);
            Thread.sleep(10);
        }
        return null;
    }
}
