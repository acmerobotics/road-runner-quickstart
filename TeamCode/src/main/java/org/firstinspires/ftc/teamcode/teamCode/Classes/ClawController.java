package org.firstinspires.ftc.teamcode.teamCode.Classes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ClawController {
    Servo ClawLeft, ClawRight;

    public static double LeftOpen = 0.5;
    public static double LeftClose = 0.6;
    public static double RightOpen = 0.5;
    public static double RightClose = 0.6;
    public static double LeftArrange= 0.9;

    public ClawController(HardwareMap map)
    {
        ClawLeft = map.get(Servo.class, "clawLeft");
        ClawRight = map.get(Servo.class, "clawRight");
    }


    enum Status {
        INTAKE,
        DROP,
        ARRANGE,
        OPEN,
        CLOSE
    }
    public Status LeftStatus, RightStatus;
    LeftStatus=Status.OPEN;

    public void goToIntake() {
        ClawLeft.setPosition(LeftOpen);
        ClawRight.setPosition(RightOpen);
    }

    public void goToPlace() {
        ClawLeft.setPosition(LeftClose);
        ClawRight.setPosition(RightClose);
    }

    public void goToArrange() {
        ClawLeft.setPosition(LeftArrange);
        ClawRight.setPosition(RightClose);
    }

}
