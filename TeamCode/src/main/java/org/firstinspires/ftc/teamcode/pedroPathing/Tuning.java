package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.Tests;

/** AutoTune 1.0.0 discovers static zero-argument factories returning Procedure exactly. */
public final class Tuning {
    private Tuning() { }

    @Tuner(name = "8103 — 1. Mecanum setup")
    public static Procedure mecanum() { return new MecanumTuner(); }

    @Tuner(name = "8103 — 2. Pinpoint setup")
    public static Procedure pinpoint() { return new PinpointTuner(); }

    @Tuner(name = "8103 — 3. Foresight calibration")
    public static Procedure foresight() { return new ForesightTuner(); }

    @Tuner(name = "8103 — 4. Path and localization tests")
    public static Procedure tests() { return new Tests(); }
}
