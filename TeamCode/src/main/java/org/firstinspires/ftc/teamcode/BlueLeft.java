package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Config
@Autonomous(name = "BlueLeft", group = "robot", preselectTeleOp = "FluffyTeleOp")
public class BlueLeft extends LinearOpMode {
    //BRING GRABBER UP FIRST THING
    AutoFluffy fluffy;
    String PATH;
    public static double DeltaC_X = 3.86;
    public static double DeltaC_Y = 3.51;

    public static double Target_X;
    public static double Target_Y;

    AprilTagDetection detection;
    public static double delta = 1;
    List<Recognition> currentRecognitions;
    public void runOpMode(){
        initialize();
        while(!isStarted() && !isStopRequested()){
            PATH= fluffy.getPropLocation();
            telemetry.addData("Prop Location", PATH );
            telemetry.addData("Left Sat. Value", fluffy.getLeftMean());
            telemetry.addData("Center Sat. Value", fluffy.getCenterMean());
            telemetry.addData("Right Sat. Value", fluffy.getRightMean());
            telemetry.update();
            sleep(5000);

        }

        // JRC: Turn off redFinder at this point.

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
        deliverYellowTest();
        //deliverYellow();
    }


    public void initialize(){
        fluffy = new AutoFluffy(this, "Blue");
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
                        .strafeToLinearHeading(new Vector2d(36.25, 4), Math.toRadians(-90))
                        //.turnTo(Math.toRadians(-91))
                        .build());
        fluffy.deliverPurple();
    }
    public void deliverPurpleRight(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)

        public void yellowLeft(){
                        .strafeToLinearHeading(new Vector2d(26,-5), Math.toRadians(-90))
                        .build());
        fluffy.deliverPurple();
    }
        /* JRC: To read position, use
         * fluffy.drive.pose.position.x, fluffy.drive.pose.position.y
         * fluffy.drive.pose.heading.toDouble();
         * Don't forget to handle the null case gracefully!
         */
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(new Vector2d(34.4, -26), Math.toRadians(-90))
                        .build());
    }
    public void yellowCenter(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo(new Vector2d(30, -.5))
                        .strafeToLinearHeading(new Vector2d(28.4, -26), Math.toRadians(-90))
                        .build());
    }
    public void yellowRight(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(new Vector2d(18, -5), Math.toRadians(-90))
                        .strafeToLinearHeading(new Vector2d(22.4, -26), Math.toRadians(-90))
                        .build());
    }

    public void deliverYellowTest(){
        telemetry.addData("Current position", fluffy.drive.pose);
        telemetry.addData("Destination", fluffy.correctYellowPosition(PATH));
        telemetry.update();
        sleep(5000);
        Pose2d pose = fluffy.correctYellowPosition(PATH);
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(pose.position , pose.heading)
                        .build());
    }

    public void deliverYellow(){
        fluffy.raiseLift();
        fluffy.correctYellowPosition(PATH);
        fluffy.raiseFinger();
        telemetry.addData("correctYellowPosition", fluffy.correctYellowPosition(PATH));
        sleep(5000);
        /* HA plan 12/13:
         * raiseLift()
         * get detection  // JRC - see below
         * do math      // JRC - math method is fluffy.correctYellowPosition, gets detection within.
         * create trajectory and drive to it
         * raiseFinger()
         * back up
         * lowerLift()
         * NOTE: must park after delivering yellow.
         */

        /* JRC testing recommendation
         * write deliverYellow with raiseLift, correctYellowPosition, raiseFinger, grabberDown.  (ie, no driving).
         *    Test just that, and use telemetry to determine whether correctYellowPosition is working properly.
         * When satisfied, add driving.  Test final version.
         * (then park)
         */

    }


}