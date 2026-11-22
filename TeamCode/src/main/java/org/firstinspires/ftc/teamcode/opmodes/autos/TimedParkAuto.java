package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.function.LongSupplier;

/** One robot-relative move from a repeatable wall start. Tune on the practice field. */
@Configurable
@Autonomous(name = "Timed Leave + Park", group = "Competition")
public class TimedParkAuto extends TimedDriveAuto {
    public static volatile int moveMs = 500;
    public static volatile double forwardPower = 0.25, leftPower = 0;

    public TimedParkAuto() { this(System::nanoTime); }

    TimedParkAuto(LongSupplier clock) { super(clock); }

    @Override public void init() {
        initMove(moveMs, forwardPower, leftPower, "Ready. Line up at the practiced wall start.");
    }
}
