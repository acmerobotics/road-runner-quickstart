package org.firstinspires.ftc.teamcode.teamCode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawController {
    Servo left, right;
    double leftClosed = 0.6;
    double leftOpen = 0.45;
    double rightClosed = 0.65;
    double rightOpen = 0.8;

    public enum Status
    {
        OPEN,
        CLOSED
    }

    Status leftStatus = Status.OPEN;
    Status rightStatus = Status.OPEN;

    public ClawController (HardwareMap map) {
        left = map.get(Servo.class, "s0e");
        right = map.get(Servo.class, "s2e");
    }
    public void toggleLeft() {
        if(leftStatus == Status.OPEN) {
            leftStatus = Status.CLOSED;
            left.setPosition(leftClosed);
        } else {
            leftStatus = Status.OPEN;
            left.setPosition(leftOpen);
        }
    }

    public void toggleRight() {
        if(rightStatus == Status.OPEN) {
            rightStatus = Status.CLOSED;
            right.setPosition(rightClosed);
        } else {
            rightStatus = Status.OPEN;
            right.setPosition(rightOpen);
        }
    }
}
