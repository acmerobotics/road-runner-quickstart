package org.firstinspires.ftc.teamcode.teleop.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "competition")
@Config
public class IntakeTest extends LinearOpMode {
    CRServo servo;
    CRServo servo2;
    public static double SPEED = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(CRServo.class, "claw1");
        servo2 = hardwareMap.get(CRServo.class, "claw2");
        servo2.setDirection(DcMotorSimple.Direction.REVERSE);
        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            servo.setPower(SPEED);
            servo2.setPower(SPEED);
        }
    }
}