package org.firstinspires.ftc.teamcode.teamCode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class JointController {
    Servo left, right;
    double downPoz = 0.5;
    double midPoz = 0.5;
    double upPoz = 0.5;

    public JointController (HardwareMap map) {
        left = map.get(Servo.class, "");
        right = map.get(Servo.class, "");
    }
}
