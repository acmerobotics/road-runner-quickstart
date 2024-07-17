package org.firstinspires.ftc.teamcode.teamCode.Classes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Outtake4Bar {
    Servo left, right;

    public static double IntakePos = 0.5;
    public static double DropPos = 0.5;
    public static double ReadyPos = 0.5;

    public Outtake4Bar(HardwareMap map)
    {
        left = map.get(Servo.class, "");
        right = map.get(Servo.class, "");
    }

    enum Status {
        INTAKE,
        DROP
    }

    public void setPosition(double pos) {
        left.setPosition(pos);
        right.setPosition(pos);
    }

    public void goToIntake() {
        setPosition(IntakePos);
    }

    public void goToReady() {
        setPosition(ReadyPos);
    }

    public void goToDrop() {
        setPosition(DropPos);
    }
}
