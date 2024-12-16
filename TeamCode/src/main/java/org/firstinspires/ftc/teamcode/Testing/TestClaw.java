package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestClaw extends LinearOpMode {
    @Override
    public void runOpMode() {

        Servo claw = hardwareMap.get(Servo.class, "clawServo");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                claw.setPosition(0.4);
            }

            if (gamepad1.x) {
                claw.setPosition(0.2);
            }

            if (gamepad1.b) {
                claw.setPosition(0);
            }


        }
    }

}
