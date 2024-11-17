package org.firstinspires.ftc.teamcode.teleop.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "competition")
@Config
public class ServoZero extends LinearOpMode {
    Servo servo;
    Servo servo2;
    public static double DEP_POS = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "arm1");
        servo2 = hardwareMap.get(Servo.class, "arm2");
        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            servo.setPosition(DEP_POS);
        }
    }
}