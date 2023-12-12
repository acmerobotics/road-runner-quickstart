package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;
@Config
@Autonomous(name = "RedRight", group = "robot", preselectTeleOp = "FluffyTeleOp")
public class RedRight extends LinearOpMode {
    //BRING GRABBER UP FIRST THING
    AutoFluffy fluffy;
    public static String PATH = "Center";
    public static double delta = 1;
    List<Recognition> currentRecognitions;
    public void runOpMode(){
        initialize();
        while(!isStarted() && !isStopRequested()){
            //currentRecognitions=fluffy.getRecognitions();
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
            //sleep(5000);*/
            //PATH= fluffy.getSide();
            telemetry.addData("Side: ", PATH );
            telemetry.update();


        }

        if(PATH == "Left"){
            deliverPurpleLeft();
            yellowLeft();
        } else if(PATH == "Right"){
            deliverPurpleRight();
            yellowRight();
        }else{
            deliverPurpleCenter();
            yellowCenter();
        }

        telemetry.addData("x", fluffy.drive.pose.position.x);
        telemetry.addData("y", fluffy.drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(fluffy.drive.pose.heading.toDouble()));
        telemetry.update();
        sleep(5000);

    }


    public void initialize(){
        fluffy = new AutoFluffy(this, "Red");
        fluffy.init();
        fluffy.raiseGrabber();
    }

    public void deliverPurpleLeft(){
        Actions.runBlocking(
           fluffy.drive.actionBuilder(fluffy.drive.pose)
                   //.splineTo(new Vector2d(29.5,9.23), .21)
                   .lineToX(20.5)
                   .strafeTo( new Vector2d(29.5,10.5))
                   .build());
        fluffy.deliverPurple();

    }
    public void deliverPurpleCenter(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        //.splineTo(new Vector2d(27.93, -10.81), -91.4)
                        //.lineToX(29)
                        .strafeToLinearHeading(new Vector2d(36.25, -.5), Math.toRadians(-90))
                        //.turnTo(Math.toRadians(-91))
                        .build());
        fluffy.deliverPurple();
    }
    public void deliverPurpleRight(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        //.splineTo(new Vector2d(25, -15), -90)
                        //.lineToX(12)
                        //.lineToXLinearHeading(20, Math.toRadians(-90))
                        .strafeToLinearHeading(new Vector2d(20,-5), Math.toRadians(-90))
                        //.turnTo(Math.toRadians(-90))
                        //.lineToY(-15)
                        //.lineToY(-15)
                        .build());
        fluffy.deliverPurple();
    }

    public void yellowLeft(){
        Actions.runBlocking(
              fluffy.drive.actionBuilder(fluffy.drive.pose)
                      .strafeToLinearHeading(new Vector2d(34.4, -26), Math.toRadians(-90))
                      .build());
    }
    public void yellowCenter(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(new Vector2d(34.4, -31), Math.toRadians(-90))
                        .build());
    }
    public void yellowRight(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(new Vector2d(34.4, -38), Math.toRadians(-90))
                        .build());
    }

    public void deliverYellow(){
        fluffy.raiseLift();
        sleep (5000); //fix values
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .lineToY(fluffy.drive.pose.position.y - 6)
                        .build()); //fix values
        fluffy.raiseFinger();
        sleep (300); //fix values

        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .setReversed(true)
                        .lineToY(fluffy.drive.pose.position.y + 6)
                        .setReversed(false)
                        .build()); //fix values
        fluffy.grabberDown();
        sleep(300); //fix values
        fluffy.lowerLift();
        sleep(5000); //fix values

    }

}
