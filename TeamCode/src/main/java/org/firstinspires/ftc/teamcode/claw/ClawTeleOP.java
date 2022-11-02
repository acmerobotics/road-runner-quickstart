package org.firstinspires.ftc.teamcode.claw;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "drive")
public class ClawTeleOP extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo clawServo = hardwareMap.get(Servo.class, "clawServo");
        Claw claw = new Claw(clawServo);
        waitForStart();
        boolean toggleClaw = true;
        while (!isStopRequested()) {
            /*int x = 1
            while (x = 1){ *//**/
            if (gamepad1.a) {
                if (toggleClaw) {
                    toggleClaw = false;
                    claw.clawOpen();
                } else {
                    toggleClaw = true;
                    claw.clawClose();
                }
            }
        }
    }
}

