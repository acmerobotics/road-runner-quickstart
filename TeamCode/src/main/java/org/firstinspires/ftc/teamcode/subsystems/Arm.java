package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Mechanism;


public class Arm extends Mechanism {
    Servo leftArm;
    Servo rightArm;
    public String leftName = "leftArm";
    public String rightName = "rightArm";
    double extendedPosition = 1;
    double retractedPosition = 0;

    public boolean isExtended = false;

    public boolean isRetracted = false;

    @Override
    public void init(HardwareMap hwMap) {
        leftArm = hwMap.get(Servo.class, leftName);
        rightArm = hwMap.get(Servo.class, rightName);
    }

    @Override
    public void loop(Gamepad gamepad) {
        isExtended = leftArm.getPosition() == extendedPosition && rightArm.getPosition() == extendedPosition;
        isRetracted = leftArm.getPosition() == retractedPosition && rightArm.getPosition() == retractedPosition;
    }

    public void extend() {
        leftArm.setPosition(extendedPosition);
        rightArm.setPosition(extendedPosition);
    }

    public void stage() {
        leftArm.setPosition(retractedPosition);
        rightArm.setPosition(retractedPosition);
    }
}
