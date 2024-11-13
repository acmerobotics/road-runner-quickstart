package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestServo extends LinearOpMode {
    @Override
    public void runOpMode() {

        Servo left = hardwareMap.get(Servo.class, "clawLeftServo");
        Servo right = hardwareMap.get(Servo.class, "clawRightServo");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                left.setPosition(0.2);
                right.setPosition(0.8);
            }

            if (gamepad1.x) {
                left.setPosition(0.5);
                right.setPosition(0.5);
            }

            if (gamepad1.b) {
                left.setPosition(0.55);
                right.setPosition(0.45);
            }


        }
    }

}
