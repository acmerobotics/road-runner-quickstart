package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
/* TODO
   - turn off redFinder when no longer needed.
 */
@Config
@Autonomous(name = "RedLeft", group = "robot", preselectTeleOp = "FluffyTeleOp")
public class RedLeft extends LinearOpMode {
    //BRING GRABBER UP FIRST THING
    AutoFluffy fluffy;
    String PATH;
    String SIDE = "Left";
    public static double DeltaC_X = -3.86;
    public static double DeltaC_Y = -3.51;
    static double DELTA = 1;

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
        }

        // JRC: Turn off redFinder at this point.

        if(PATH == "Left"){
            deliverPurpleLeft();
            yellowDriveUp();
            //yellowLeft();
        } else if(PATH == "Right"){
            deliverPurpleRight();
            yellowDriveUp();
            //yellowRight();
        }else{
            deliverPurpleCenter();
            yellowDriveUp();
            //yellowCenter();
        }
        sleep(2000);
        deliverYellow();
        park();
        //RobotLog.i(String.format("Final position %f,%f",fluffy.drive.pose.position.x, fluffy.drive.pose.position.y));
    }


    public void initialize(){
        fluffy = new AutoFluffy(this, "Red");
        fluffy.raiseGrabber();
    }

    public void deliverPurpleLeft(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo( new Vector2d(35,10))
                        //.strafeTo( new Vector2d(35,30))
                        .strafeTo( new Vector2d(35,29.5))
                        //.strafeTo( new Vector2d(25,26))
                        .build());
        fluffy.deliverPurple();
        fluffy.retractPurple();
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo( new Vector2d(35,-1))
                        .build());

    }
    public void deliverPurpleCenter(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(new Vector2d(38, 20 ), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(41.5, 18.5 ), Math.toRadians(0))
                        .build());
        fluffy.deliverPurple();
        fluffy.retractPurple();
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(new Vector2d(39, 25 ), Math.toRadians(0))
                        .build());
    }
    public void deliverPurpleRight(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(new Vector2d(27.6,6), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(29.0,-5), Math.toRadians(0))
                        .build());
        fluffy.deliverPurple();
        fluffy.retractPurple();
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(new Vector2d(28.0,6), Math.toRadians(0))
                        .build());
    }

    public void yellowLeft(){
        /* JRC: To read position, use
         * Don't forget to handle the null case gracefully!
         */
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(new Vector2d(34.4, -26), Math.toRadians(-89.9))
                        .build());
    }
    public void yellowCenter(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo(new Vector2d(30, -.5))
                        .strafeToLinearHeading(new Vector2d(28.4, -26), Math.toRadians(-89.9))
                        .build());
    }
    public void yellowRight(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(new Vector2d(18, -5), Math.toRadians(-89.9))
                        .strafeToLinearHeading(new Vector2d(22.4, -26), Math.toRadians(-89.9))
                        .build());
    }

    public void yellowDriveUp(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(new Vector2d(56, 13.76), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(50, -49.8), Math.toRadians(-89.9))
                        .strafeToLinearHeading(new Vector2d(19.38, -78), Math.toRadians(-89.9))
                        .build());
    }

    public void deliverYellow(){
        fluffy.drive.pose = new Pose2d (0,0,-89.9);
        fluffy.raiseLift();
        Pose2d destination = fluffy.correctYellowPositionRed(PATH, SIDE);
        RobotLog.i(String.format("Target position: %s", PATH));
        RobotLog.i(String.format("Current position: (%3.1f, %3.1f) at %3.1f deg",
                fluffy.drive.pose.position.x,
                fluffy.drive.pose.position.y,
                Math.toDegrees(fluffy.drive.pose.heading.toDouble())));
        RobotLog.i(String.format("Destination position: (%3.1f, %3.1f) at %3.1f deg",
                destination.position.x,
                destination.position.y,
                Math.toDegrees(destination.heading.toDouble())));
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(destination.position , destination.heading)
                        .build());
        RobotLog.i(String.format("New current position: (%3.1f, %3.1f) at %3.1f deg",
                fluffy.drive.pose.position.x,
                fluffy.drive.pose.position.y,
                Math.toDegrees(fluffy.drive.pose.heading.toDouble())));
        fluffy.raiseFinger();
        sleep(500);
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(new Vector2d(fluffy.drive.pose.position.x, (fluffy.drive.pose.position.y + DELTA)),
                                fluffy.drive.pose.heading)
                        .build());
        fluffy.lowerLift();

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
    public void park(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(new Vector2d(15, -5), fluffy.drive.pose.heading)
                        .build());
    }


}
