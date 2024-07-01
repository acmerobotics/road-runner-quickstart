package org.firstinspires.ftc.teamcode.teamCode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawController {
    Servo left, right;
    double leftClosed = 0.5;
    double leftOpen = 0.5;
    double rightClosed = 0.5;
    double rightOpen = 0.5;

    public enum Status
    {
        OPEN,
        CLOSED
    }

    Status leftStatus = Status.OPEN;
    Status rightStatus = Status.OPEN;

    public ClawController (HardwareMap map) {
        left = map.get(Servo.class, "");
        right = map.get(Servo.class, "");
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
