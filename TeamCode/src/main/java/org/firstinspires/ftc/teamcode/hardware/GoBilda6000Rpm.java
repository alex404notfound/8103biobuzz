package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

/** 5203-2402-0001: https://www.gobilda.com/content/user_manuals/5203-2402-0001_spec_sheet.pdf */
@MotorType(ticksPerRev = 28, gearing = 1, maxRPM = 6000,
        achieveableMaxRPMFraction = 1.0, orientation = Rotation.CCW)
@DeviceProperties(name = "goBILDA 6000 RPM (1:1)", xmlTag = "goBILDA6000RpmMotor")
public interface GoBilda6000Rpm { }
