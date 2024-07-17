package org.firstinspires.ftc.teamcode.teamCode.Classes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class OuttakeJoint {
    Servo servo;

    public static double IntakePos = 0.5;
    public static double ReadyPos = 0.5;
    public static double DropPos = 0.5;

    public OuttakeJoint(HardwareMap map)
    {
        servo = map.get(Servo.class, "");
    }

    enum Status {
        INTAKE,
        DROP,
        READY
    }

    public void goToIntake()
    {
        servo.setPosition(IntakePos);
    }
    public void goToDrop()
    {
        servo.setPosition(DropPos);
    }
    public void goToReady()
    {
        servo.setPosition(ReadyPos);
    }

}
