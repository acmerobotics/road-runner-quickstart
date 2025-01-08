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

public class DriveToTouchBar extends LinearOpMode
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
        robot.setStatus("Moving forward");
        robot.driveTrain.moveInDirectForDistance(.5,0,1,48);//forward
        robot.setStatus("turning");
        robot.driveTrain.turn(-90);
        robot.setStatus("Moving right");
        robot.driveTrain.moveInDirectForDistance(.5,0,1,10);
        robot.sleep(30*1000);
    }
}
