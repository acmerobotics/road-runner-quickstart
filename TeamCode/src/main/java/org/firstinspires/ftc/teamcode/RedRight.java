package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.AutoFluffy.GRABBER_UP;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@Config
@Autonomous(name = "RedRight", group = "robot")
public class RedRight extends LinearOpMode {
    //BRING GRABBER UP FIRST THING
    AutoFluffy fluffy;
    public static String PATH = "Center";
    List<Recognition> currentRecognitions;
    public void runOpMode(){
        initialize();
        while(!isStarted() && !isStopRequested()){
            currentRecognitions=fluffy.getRecognitions();
            if(gamepad1.x){
                PATH = "Left";
                telemetry.addData("TargetPosition: ", PATH);
            }
            if(gamepad1.y){
                PATH = "Center";
                telemetry.addData("TargetPosition: ", PATH);
            }
            if(gamepad1.b){
                PATH = "Right";
                telemetry.addData("TargetPosition: ", PATH);
            }
            //telemetry.addData("TargetPosition: ", PATH);
            telemetry.update();
            sleep(5000);


        }

        if(PATH == "Left"){
            deliverPurpleLeft();
        } else if(PATH == "Right"){
            deliverPurpleRight();
        }else{
            deliverPurpleCenter();
        }

        telemetry.addData("x", fluffy.drive.pose.position.x);
        telemetry.addData("y", fluffy.drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(fluffy.drive.pose.heading.toDouble()));
        telemetry.update();
        sleep(20000);
    }
    public void initialize(){
        fluffy = new AutoFluffy(this, "Red");
        fluffy.raiseGrabber();
    }

    public void deliverPurpleLeft(){
        Actions.runBlocking(
           fluffy.drive.actionBuilder(fluffy.drive.pose)
                   .splineTo(new Vector2d(29.5, -9.23), .21)
                   .build());

    }
    public void deliverPurpleCenter(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .splineTo(new Vector2d(27.93, -10.81), -91.4)
                        .build());
    }
    public void deliverPurpleRight(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .splineTo(new Vector2d(30.12, -2.61), 177.5)
                        .build());
    }


}
