package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class BasicDrive extends LinearOpMode {

    public RobotDrive bot;

    @Override
    public void runOpMode() {
        bot = new RobotDrive();
        bot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double jx = -gamepad1.left_stick_y - gamepad1.right_stick_y;
            double jy = -gamepad1.left_stick_x;
            double jw = -gamepad1.right_stick_x;

            double speed = (1.0 - gamepad1.left_trigger * 0.75);

            bot.driveXYW(jx * speed, jy * speed, jw);

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
