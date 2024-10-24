package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestServo extends LinearOpMode {
    private Servo servo1;
    @Override
    public void runOpMode() {

        servo1 = hardwareMap.get(Servo.class, "servo1");


        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                servo1.setPosition(0.5);
            }

            if (gamepad1.b) {
                servo1.setPosition(1);
            }

            if (gamepad1.x) {
                servo1.setPosition(0);
            }

        }
    }

}
