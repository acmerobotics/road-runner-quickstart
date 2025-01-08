package org.firstinspires.ftc.teamcode.onbotjava;
//import org.firstinspires.ftc.teamcode.vision.*;
import com.qualcomm.robotcore.robot.Robot;

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

@TeleOp

public class TeleOp2 extends LinearOpMode
{
    private Robot2024 robot;
    //USE : returns moter power scale
    //USE : init
    public void runOpMode()
    {
        robot = new Robot2024(this,false);
        //Wait for play
        while (!isStarted())
        {
           robot.loop();
        }
       
        while (!isStopRequested())
        {
            robot.teleopLoop();
            robot.loop();
        }
    }
}

// Plan for robot movement

// Step 1: Take in the infro of how fast each motor should move. 
//         Use getMotorPowersForAngleInQuadrant to do this.

// Step 2: Take this information and give the correct motor power
//         to each individual motor

// Step 3: Pray
