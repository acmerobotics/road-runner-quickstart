package org.firstinspires.ftc.teamcode.configs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Config
public class TestConfigMotor1 extends LinearOpMode {
    DcMotorEx motor;
    public static double speed = 0;
    public static String name = "";
    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.get(DcMotorEx.class, name);
        waitForStart();
        while(opModeIsActive())
        {
            motor.setPower(speed);
        }

    }
}
