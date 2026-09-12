package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;

/**
 * The copy-paste starting point for every new mechanism (spec §4.2 anatomy):
 *
 *  1. @Config class — the public static fields below are live-editable in the
 *     FTC Dashboard (http://192.168.43.1:8080/dash) while an opmode runs.
 *  2. Constructor takes the hardware map and telemetry; hardware uses a name
 *     matching src/main/res/xml/teamconfig8103.xml verbatim.
 *  3. A Mode enum + switch inside periodic() — API methods only change the mode;
 *     periodic() writes actuator power during operation; stop() writes a safe output.
 *  4. periodic() is an ordinary control/telemetry update called by Robot once
 *     per cycle. Never block in it. stop() explicitly makes the actuator safe.
 *  5. Ivy commands declare .requiring(this) to arbitrate actions for this mechanism.
 *     on()/off() set a persistent mode; runWhileScheduled() owns its running lifetime.
 *
 * Register construction, periodic(), and stop() together in Robot.
 */
@Config
public class ExampleSubsystem {
    public static double onPower = 1;
    public static double offPower = 0;

    private final DcMotorEx motor;
    private final Telemetry telemetry;

    private enum Mode {ON, OFF}

    private Mode mode = Mode.OFF;

    public ExampleSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        motor = hardwareMap.get(DcMotorEx.class, "example");
        this.telemetry = telemetry;
    }

    /** Sets ON and finishes immediately; motor remains on until another action changes the mode. */
    public Command on() {
        return instant(() -> mode = Mode.ON).requiring(this);
    }

    public Command off() {
        return instant(() -> mode = Mode.OFF).requiring(this);
    }

    public Command toggle() {
        return conditional(() -> mode == Mode.OFF, on(), off());
    }

    /** Runs until canceled, interrupted, or ended by a composition such as race(...). */
    public Command runWhileScheduled() {
        return Command.build()
                .setStart(() -> mode = Mode.ON)
                .setDone(() -> false)
                .setEnd(condition -> mode = Mode.OFF)
                .requiring(this);
    }

    public void periodic() {
        switch (mode) {
            case ON:
                motor.setPower(onPower);
                break;
            case OFF:
                motor.setPower(offPower);
                break;
        }
        telemetry.addData("Example Mode", mode);
        telemetry.addData("Example Velocity", motor.getVelocity());
    }

    public void stop() {
        mode = Mode.OFF;
        motor.setPower(0);
    }
}
