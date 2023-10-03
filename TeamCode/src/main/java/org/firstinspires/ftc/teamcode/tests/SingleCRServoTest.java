package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="SingleCRServoTest", group="Tests")
public class SingleCRServoTest extends OpMode {
    // A teleOp to test the a single CRServo

    CRServo servo;

    @Override
    public void init() {
        servo = hardwareMap.get(CRServo.class, "servo");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            servo.setPower(1);
        } else if (gamepad1.b) {
            servo.setPower(-1);
        } else {
            servo.setPower(0);
        }
    }
}
