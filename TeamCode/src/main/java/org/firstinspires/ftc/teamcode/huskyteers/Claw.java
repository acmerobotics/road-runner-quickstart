package org.firstinspires.ftc.teamcode.huskyteers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public final class Claw {
    public final Servo clawRotator, clawGrabber;

    public Claw(HardwareMap hardwareMap) {
        clawRotator = hardwareMap.get(Servo.class, "claw_rotator");
        clawGrabber = hardwareMap.get(Servo.class, "claw_grabber");
    }
}
