package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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
    double extendedPosition = .28;
    double retractedPosition = 0.04;
    double safeRetractedPosition = 0.04;

    public boolean isExtended = false;
    public boolean isRetracted = false;
    public boolean isSafeRetracted = false;

    @Override
    public void init(HardwareMap hwMap) {
        leftArm = hwMap.get(Servo.class, leftName);
        rightArm = hwMap.get(Servo.class, rightName);
        rightArm.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop(Gamepad gamepad) {
        isExtended = Math.abs(leftArm.getPosition() - extendedPosition) < 0.01 && Math.abs(rightArm.getPosition() - extendedPosition) < 0.01;
        isRetracted = Math.abs(leftArm.getPosition() - retractedPosition) < 0.01 && Math.abs(rightArm.getPosition() - retractedPosition) < 0.01;
        isSafeRetracted = Math.abs(leftArm.getPosition() - safeRetractedPosition) < 0.01 && Math.abs(rightArm.getPosition() - safeRetractedPosition) < 0.01;
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Left Arm Position:", leftArm.getPosition());
        telemetry.addData("Right Arm Position:", rightArm.getPosition());
        telemetry.addData("isExtended: ", isExtended);
        telemetry.addData("isRetracted: ", isRetracted);
    }

    public void extend() {
        leftArm.setPosition(extendedPosition);
        rightArm.setPosition(extendedPosition);
    }

    public void retract() {
        leftArm.setPosition(retractedPosition);
        rightArm.setPosition(retractedPosition);
    }

    public void safeRetract() {
        leftArm.setPosition(safeRetractedPosition);
        rightArm.setPosition(safeRetractedPosition);
    }

    public void setPower(double input) {
        leftArm.setPosition(extendedPosition + input);
        rightArm.setPosition(extendedPosition + input);
    }
}
