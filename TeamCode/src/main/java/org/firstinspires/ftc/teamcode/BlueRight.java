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
@Autonomous(name = "BlueRight", group = "robot", preselectTeleOp = "FluffyTeleOp")
public class BlueRight extends LinearOpMode {
    //BRING GRABBER UP FIRST THING//
    AutoFluffy fluffy;
    String PATH;
    String SIDE = "Right";
    static double DELTA = 2;
    List<Recognition> currentRecognitions;
    public void runOpMode(){
        initialize();
        while(!isStarted() && !isStopRequested()){
            //currentRecognitions=fluffy.getRecognitions();
            /*if(gamepad1.x){
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
            }*/
            //telemetry.addData("TargetPosition: ", PATH);
            // telemetry.update();
            PATH= fluffy.getPropLocation();
            telemetry.addData("Prop Location", PATH );
            telemetry.addData("Left Sat. Value", fluffy.getLeftMean());
            telemetry.addData("Center Sat. Value", fluffy.getCenterMean());
            telemetry.addData("Right Sat. Value", fluffy.getRightMean());

            telemetry.update();


        }

        // JRC: Turn off redFinder at this point.
        sleep(2000);
        if(PATH .equals("Left")){
            deliverPurpleLeft();
            yellowLeft();
        } else if(PATH .equals("Right")){
            deliverPurpleRight();
            yellowRight();
        }else{
            deliverPurpleCenter();
            yellowCenter();
        }
        sleep(2000);
        deliverYellow();
        park();
        //RobotLog.i(String.format("Final position %f,%f",fluffy.drive.pose.position.x, fluffy.drive.pose.position.y));
    }


    public void initialize(){
        fluffy = new AutoFluffy(this, "Blue");
        fluffy.raiseGrabber();
    }

    public void deliverPurpleLeft(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(new Vector2d(27.6,-6), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(29.0,7.75), Math.toRadians(0))
                        .build());
        fluffy.deliverPurple();
        fluffy.retractPurple();
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(new Vector2d(28.0,-6), Math.toRadians(0))
                        .build());
    }
    public void deliverPurpleCenter() {
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(new Vector2d(38, -18 ), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(41.5, -9 ), Math.toRadians(0))
                        .build());
        fluffy.deliverPurple();
        fluffy.retractPurple();
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(new Vector2d(39, -15 ), Math.toRadians(0))
                        .build());
    }
    public void deliverPurpleRight(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo( new Vector2d(32.5,5))
                        //.strafeTo( new Vector2d(35,30))
                        .strafeTo( new Vector2d(32.5,-16.5))
                        //.strafeTo( new Vector2d(25,26))
                        .build());
        fluffy.deliverPurple();
        fluffy.retractPurple();
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo( new Vector2d(32.5,-20))
                        .build());
    }

    public void yellowLeft(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(new Vector2d(56, -7.76), Math.toRadians(0))
                        .turnTo(Math.toRadians(89.9))
                        //.strafeToLinearHeading(new Vector2d(56, 13.76), Math.toRadians(-89.9))
                        .lineToY(70.8)
                        .strafeToLinearHeading(new Vector2d(26, 84), Math.toRadians(89.9))
                        .build());
    }
    public void yellowCenter(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(new Vector2d(56, -7.76), Math.toRadians(0))
                        .turnTo(Math.toRadians(89.9))
                        //.strafeToLinearHeading(new Vector2d(56, 13.76), Math.toRadians(-89.9))
                        .lineToY(70.8)
                        .strafeToLinearHeading(new Vector2d(30, 84), Math.toRadians(89.9))
                        .build());
    }
    public void yellowRight(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(new Vector2d(56, -7.76), Math.toRadians(0))
                        .turnTo(Math.toRadians(89.9))
                        //.strafeToLinearHeading(new Vector2d(56, 13.76), Math.toRadians(-89.9))
                        .lineToY(70.8)
                        .strafeToLinearHeading(new Vector2d(34, 84), Math.toRadians(89.9))
                        .build());
    }

    public void yellowDriveUp(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(new Vector2d(56, -7.76), Math.toRadians(0))
                        .turnTo(Math.toRadians(89.9))
                        //.strafeToLinearHeading(new Vector2d(56, 13.76), Math.toRadians(-89.9))
                        .lineToY(70.8)
                        .strafeToLinearHeading(new Vector2d(30, 84), Math.toRadians(89.9))
                        .build());
    }

    public void deliverYellow(){
        fluffy.raiseLift();
        Pose2d destination = fluffy.correctYellowPositionBlue(PATH, SIDE);
        RobotLog.i(String.format("Destination position: (%3.1f, %3.1f) at %3.1f deg",
                destination.position.x,
                destination.position.y,
                Math.toDegrees(destination.heading.toDouble())));
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(destination.position , destination.heading)
                        .build());
        fluffy.raiseFinger();
        sleep(500);
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(new Vector2d(fluffy.drive.pose.position.x, (fluffy.drive.pose.position.y - DELTA)),
                                Math.toRadians(89.9))
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
                        .strafeToLinearHeading(new Vector2d(5, 86), Math.toRadians(89.9))
                        .build());
    }


}

