package org.firstinspires.ftc.teamcode.teamCode.Classes;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeRotation {
    public Servo servo;
    double Servo_Level = 0.5;
    double Servo_Left = 0.5;
    double Servo_Right = 0.5;

    public OuttakeRotation(HardwareMap map)
    {
        servo = map.get(Servo.class, "");
    }

    enum Status {
        LEVEL,
        LEFT,
        RIGHT
    }

    public void goToLevel()
    {
        servo.setPosition(Servo_Left);
    }
    public void goLeft()
    {
        servo.setPosition(Servo_Level);
    }
    public void goRight()
    {
        servo.setPosition(Servo_Right);
    }
}
