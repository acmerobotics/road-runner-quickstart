package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class ArcadeDrive extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);

        bot.leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        bot.rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        bot.leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double leftFrontPower = (y + x + rx) / denominator;
            double leftBackPower = (y - x + rx) / denominator;
            double rightFrontPower = (y - x - rx) / denominator;
            double rightBackPower = (y + x - rx) / denominator;

            bot.leftFront.setPower(leftFrontPower);
            bot.leftBack.setPower(leftBackPower);
            bot.rightFront.setPower(rightFrontPower);
            bot.rightBack.setPower(rightBackPower);
        }
    }
}
