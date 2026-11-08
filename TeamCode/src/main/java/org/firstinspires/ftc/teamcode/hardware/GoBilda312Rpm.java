package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

/** goBILDA Yellow Jacket with the 19.2:1 gearbox. Encoder ticks are per output revolution. */
@MotorType(ticksPerRev = 537.7, gearing = 19.2, maxRPM = 312, orientation = Rotation.CCW)
@DeviceProperties(name = "goBILDA 312 RPM (19.2:1)", xmlTag = "goBILDA312RpmMotor")
public interface GoBilda312Rpm { }
