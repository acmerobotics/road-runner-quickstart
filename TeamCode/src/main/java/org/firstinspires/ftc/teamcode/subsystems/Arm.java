package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Mechanism;


public class Arm extends Mechanism {
    public Servo leftArm;
    public Servo rightArm;
    public String leftName = "leftArm";
    public String rightName = "rightArm";
    double extendedPosition = 0.3;
    double retractedPosition = 0.09;

    public boolean isExtended = false;

    public boolean isRetracted = false;

    @Override
    public void init(HardwareMap hwMap) {
        leftArm = hwMap.get(Servo.class, leftName);
        rightArm = hwMap.get(Servo.class, rightName);
        leftArm.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop(Gamepad gamepad) {
        isExtended = leftArm.getPosition() == extendedPosition && rightArm.getPosition() == extendedPosition;
        isRetracted = leftArm.getPosition() == retractedPosition && rightArm.getPosition() == retractedPosition;
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Left Arm Position:", leftArm.getPosition());
        telemetry.addData("Right Arm Position:", rightArm.getPosition());
    }

    public void extend() {
        leftArm.setPosition(extendedPosition);
        rightArm.setPosition(extendedPosition);
    }

    public void retract() {
        leftArm.setPosition(retractedPosition);
        rightArm.setPosition(retractedPosition);
    }
}
