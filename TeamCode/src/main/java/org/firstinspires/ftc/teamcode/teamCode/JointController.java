package org.firstinspires.ftc.teamcode.teamCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class JointController {
    Servo left, right;
    public static double downPoz = 0.485;
    public static double midPoz = 0.4;
    public static double upPoz = 0.5;

    public JointController (HardwareMap map) {
        left = map.get(Servo.class, "s1e");
        right = map.get(Servo.class, "s3e");
    }

    public void goToMid()
    {
        left.setPosition(midPoz);
        right.setPosition(midPoz);
    }
    public void goToDown()
    {
        left.setPosition(downPoz);
        right.setPosition(downPoz);
    }
    public void goToUp()
    {
        left.setPosition(upPoz);
        right.setPosition(upPoz);
    }
}
