package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.function.LongSupplier;

@Autonomous(name = "Strafe Right Test", group = "Diagnostics")
public final class StrafeRightTestAuto extends TimedDriveAuto {
    public StrafeRightTestAuto() { this(System::nanoTime); }

    StrafeRightTestAuto(LongSupplier clock) { super(clock); }

    @Override public void init() {
        initMove(2000, 0, -0.5, "Ready to strafe right.");
    }
}
