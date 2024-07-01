package org.firstinspires.ftc.teamcode.configs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class TestConfigServo1 extends LinearOpMode {
    Servo servo;
    public static double poz = 0;
    public static String name = "";
    @Override
    public void runOpMode() throws InterruptedException {

        servo = hardwareMap.get(Servo.class, name);
        waitForStart();
        while(opModeIsActive())
        {
            servo.setPosition(poz);
        }

    }
}
