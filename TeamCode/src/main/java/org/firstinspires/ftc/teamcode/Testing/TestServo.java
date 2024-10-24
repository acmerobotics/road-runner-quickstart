package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestServo extends LinearOpMode {
    private Servo servo1;
    private Servo servo2;
    @Override
    public void runOpMode() {

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");


        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                servo1.setPosition(0.5);
                servo2.setPosition(0.5);
            }

            if (gamepad1.b) {
                servo1.setPosition(0.85);
                servo2.setPosition(0.15);
            }

            if (gamepad1.x) {
                servo1.setPosition(0);
                servo2.setPosition(1);
            }


            telemetry.addData("servo1", servo1.getPosition());
            telemetry.addData("servo2", servo2.getPosition());
            telemetry.update();
        }
    }

}
