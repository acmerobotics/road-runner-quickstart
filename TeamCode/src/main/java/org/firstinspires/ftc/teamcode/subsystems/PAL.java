package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Mechanism;

public class PAL extends Mechanism {

    Servo releaser;
    CRServo fire;
    String releaserName = "releaser";
    String fireName = "fire";

    double holdPos = 0.25;
    double releasePos = .5;
    @Override
    public void init(HardwareMap hwMap) {
        releaser = hwMap.get(Servo.class, releaserName);
        fire = hwMap.get(CRServo.class, fireName);
        releaser.setPosition(holdPos);
        fire.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (gamepad.dpad_left && gamepad.x) {
            releaser.setPosition(releasePos);
        }
        if (gamepad.dpad_up && gamepad.y) {
            fire.setPower(1);
        } else {
            fire.setPower(0);
        }
    }
}
