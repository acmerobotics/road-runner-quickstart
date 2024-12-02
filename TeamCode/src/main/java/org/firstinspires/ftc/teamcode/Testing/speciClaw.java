package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class speciClaw extends LinearOpMode {
    @Override
    public void runOpMode() {

//        Servo left = hardwareMap.get(Servo.class, "intakeServoLeft");
//        Servo right = hardwareMap.get(Servo.class, "intakeServoRight");
        Servo spClaw = hardwareMap.get(Servo.class, "spClaw");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
//                left.setPosition(0.83);
//                right.setPosition(0.83);
                spClaw.setPosition(1);
            }

            if (gamepad1.x) {
//                left.setPosition(0.5);
//                right.setPosition(0.5);
                spClaw.setPosition(0);
            }

            if (gamepad1.b) {
//                left.setPosition(0.05);
//                right.setPosition(0.05);b
                spClaw.setPosition(0.5);
            }


        }
    }

}
