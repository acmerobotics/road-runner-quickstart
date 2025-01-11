package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "Coax4Bar Test")
public class Coax4BarTest extends LinearOpMode{
    private SimpleServo Coax;
    private SimpleServo V4B;
    private GamepadEx gamepad1Ex;

    @Override
    public void runOpMode() throws InterruptedException {
        Coax = new SimpleServo(hardwareMap, "Coax", 0.0, 1.0);
        V4B = new SimpleServo(hardwareMap, "V4B", 0.0, 1.0);

        gamepad1Ex = new GamepadEx(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                Coax.setPosition(0);
            } else if (gamepad1.right_bumper) {
                Coax.setPosition(1);
            }

            if (gamepad1.a) {
                V4B.setPosition(0);
            } else if (gamepad1.b) {
                V4B.setPosition(1);
            }
        }
    }
}
