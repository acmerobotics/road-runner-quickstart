package org.firstinspires.ftc.teamcode.onbotjava;
//import org.firstinspires.ftc.teamcode.vision.*;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.Arrays;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import java.lang.Math;

@Autonomous

public class ParkRight extends LinearOpMode
{
    private Robot2024 robot;
    //USE : returns moter power scale
    //USE : init
    public void runOpMode()
    {
        robot = new Robot2024(this,true);
        //Wait for play
        while (!isStarted())
        {
           robot.loop();
        }
        
        long start_t=System.currentTimeMillis();
        robot.setStatus("Moving right");
        robot.driveTrain.moveInDirectForDistance(.5,1,0,18);
        robot.sleep(30*1000);
    }
}
