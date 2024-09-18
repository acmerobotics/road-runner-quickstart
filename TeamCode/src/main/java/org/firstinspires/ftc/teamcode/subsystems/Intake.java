package org.firstinspires.ftc.teamcode.subsystems;

import android.content.res.Configuration;

import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.settings.ConfigurationInfo;

public class Intake extends Mechanism {

    CRServo bristles;
    Servo leftHinge;
    Servo rightHinge;
    final double downHingePosition = 0;
    final double upHingePosition = 0.5;

    @Override
    public void init(HardwareMap hwMap) {
        bristles = hwMap.get(CRServo.class, ConfigurationInfo.bristles.getDeviceName());
        leftHinge = hwMap.get(Servo.class, ConfigurationInfo.leftHinge.getDeviceName());
        rightHinge = hwMap.get(Servo.class, ConfigurationInfo.rightHinge.getDeviceName());
    }

    public void intake() {
        bristles.setPower(1);
    }

    public void outtake() {
        bristles.setPower(-1);
    }

    public void hingeUp() {
        leftHinge.setPosition(upHingePosition);
        rightHinge.setPosition(upHingePosition);
    }

}
