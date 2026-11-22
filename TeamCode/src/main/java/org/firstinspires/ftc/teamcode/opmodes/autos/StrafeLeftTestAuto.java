package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.function.LongSupplier;

@Autonomous(name = "Strafe Left Test", group = "Diagnostics")
public final class StrafeLeftTestAuto extends TimedDriveAuto {
    public StrafeLeftTestAuto() { this(System::nanoTime); }

    StrafeLeftTestAuto(LongSupplier clock) { super(clock); }

    @Override public void init() {
        initMove(2000, 0, 0.5, "Ready to strafe left.");
    }
}
