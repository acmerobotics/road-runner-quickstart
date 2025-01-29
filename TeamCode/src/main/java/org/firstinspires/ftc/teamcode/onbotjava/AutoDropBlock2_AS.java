package org.firstinspires.ftc.teamcode.onbotjava;
//import org.firstinspires.ftc.teamcode.vision.*;
import com.qualcomm.robotcore.robot.Robot;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;

@Autonomous

public class AutoDropBlock2_AS extends LinearOpMode
{
    private Robot2024_AS robot;
    //USE : returns moter power scale
    //USE : init
    private Pose2D pose0=new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.DEGREES, 90);
    private Pose2D pose1=new Pose2D(DistanceUnit.INCH,-20,17.5, AngleUnit.DEGREES,90);
    private Pose2D pose2=new Pose2D(DistanceUnit.INCH,-20,17.5, AngleUnit.DEGREES,45);
    private Pose2D pose3=new Pose2D(DistanceUnit.INCH,-32,3.5, AngleUnit.DEGREES,45);//pose three is changing
    
    private Pose2D pose5=new Pose2D(DistanceUnit.INCH,-30,5, AngleUnit.DEGREES,85);
    private Pose2D pose6=new Pose2D(DistanceUnit.INCH,-32,3, AngleUnit.DEGREES,45);
    private Pose2D pose7=new Pose2D(DistanceUnit.INCH,-16,13.5, AngleUnit.DEGREES,70);
    private Pose2D pose8=new Pose2D(DistanceUnit.INCH,-21,16.5, AngleUnit.DEGREES,70);

    

    //block 1 ground
    private Pose2D pose4=new Pose2D(DistanceUnit.INCH,-21,18, AngleUnit.DEGREES,90);
    public void runOpMode()
    {
        robot = new Robot2024_AS(this,true);
        //robot.odo.setPosition(pose0);

        //Wait for play
        while (!isStarted())
        {
           robot.loop();
        }
        long start_t=System.currentTimeMillis();
            robot.arm.outtakeToCarry();
            robot.driveTrain.moveToPosition(0.5,pose0);
            //robot.waitForGamePad2X("Pose1 next.");
            robot.driveTrain.moveToPosition(0.5,pose1);
            //robot.waitForGamePad2X("Pose2 next");
            robot.driveTrain.moveToPosition(0.5,pose2);
            //robot.waitForGamePad2X("Raising lift next.");
            robot.arm.setLiftHeight(-4500);
            //robot.waitForGamePad2X("Pose3 next");
            robot.driveTrain.moveToPosition(0.35,pose3);
            //robot.waitForGamePad2X("Dropping block next.");
            robot.arm.dropBlock();
            //robot.waitForGamePad2X("Pose1 next.");
            robot.driveTrain.moveToPosition(0.35,pose1);
            //robot.waitForGamePad2X("Lowing next.");
            robot.arm.setLiftHeight(-1,false);
            //robot.sleep(1000);
            robot.driveTrain.moveToPosition(0.35,pose5);
            robot.arm.setLiftHeight(-1);

            //robot.waitForGamePad2X("Intaking next.");
            while (robot.arm.intakeMotorAS.getCurrentPosition()>-1800){
                robot.arm.intakeMotorAS.setPower(-1);
            }
            robot.arm.intakeMotorAS.setPower(0);
            robot.arm.intakeServo.setPower(1.0);
            robot.arm.backIntakeServo.setPower(-1);
            //robot.waitForGamePad2X("moving to pose4");
            robot.driveTrain.moveToPosition(0.35,pose4);
            robot.sleep(500);
            //robot.waitForGamePad2X("moving to midtake");
            while (robot.arm.intakeMotorAS.getCurrentPosition()<-50){
                robot.arm.intakeMotorAS.setPower(1);
            }
            robot.arm.intakeMotorAS.setPower(0);
            robot.arm.intakeServo.setPower(-1.0);
            robot.arm.backIntakeServo.setPower(-1);
            robot.arm.conveyorServo.setPower(-1);
            //robot.waitForGamePad2X("move to pose2");
            robot.driveTrain.moveToPosition(0.35,pose2);
            robot.sleep(2500);
            robot.arm.setServoPowersToZero();

            robot.arm.setLiftHeight(-4500);
            //robot.waitForGamePad2X("moving to pose3");
            robot.driveTrain.moveToPosition(0.35,pose3);
            //robot.waitForGamePad2X("at pose3");
            robot.arm.dropBlock();
        
            
            robot.arm.setLiftHeight(-1,false);
            //robot.waitForGamePad2X("at pose 3");
            robot.driveTrain.moveToPosition(0.5,pose7);
            //robot.waitForGamePad2X("at pose7");
            robot.arm.setLiftHeight(-1);
            while (robot.arm.intakeMotorAS.getCurrentPosition()>-1800){
                robot.arm.intakeMotorAS.setPower(-1);
            }
            robot.arm.intakeMotorAS.setPower(0);
            robot.arm.intakeServo.setPower(1.0);
            robot.arm.backIntakeServo.setPower(-1);
            robot.driveTrain.moveToPosition(0.35,pose8);
            robot.sleep(600);
            while (robot.arm.intakeMotorAS.getCurrentPosition()<-50){
                robot.arm.intakeMotorAS.setPower(1);
            }
            robot.arm.intakeMotorAS.setPower(0);
            robot.arm.intakeServo.setPower(-1.0);
            robot.arm.backIntakeServo.setPower(-1);
            robot.arm.conveyorServo.setPower(-1);
            robot.driveTrain.moveToPosition(0.35,pose2);
            robot.sleep(2500);
            robot.arm.setLiftHeight(-4500);
            robot.driveTrain.moveToPosition(0.35,pose3);
            robot.arm.setServoPowersToZero();
            robot.arm.dropBlock();
            
            robot.sleep(20*1000);
            
    }
}

