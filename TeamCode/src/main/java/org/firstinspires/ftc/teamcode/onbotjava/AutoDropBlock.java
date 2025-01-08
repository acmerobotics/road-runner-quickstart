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

public class AutoDropBlock extends LinearOpMode
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
            robot.arm.outtakeToCarry();
            robot.driveTrain.moveInDirectForDistance(0.5,0,1,10);
            robot.driveTrain.moveInDirectForDistance(0.5,-1,0,18);
            robot.driveTrain.turn(-45);
            robot.driveTrain.moveInDirectForDistance(0.5,0,-1,5);
            robot.arm.setLiftHeight(-4300);
            robot.driveTrain.moveInDirectForDistance(0.3,0,-1,3.5);
            robot.arm.outtakeToDrop();
            robot.sleep(5*1000);
            robot.arm.outtakeToStart();
            robot.arm.setLiftHeight(-500);
            robot.sleep(60*1000);
    }
}

