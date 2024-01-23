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

    final Pose2d BR_START = new Pose2d(new Vector2d(-18,60), Math.toRadians(-90));
    final Pose2d BR_CENTER_PROP_PUSH = new Pose2d(new Vector2d(-36,60), Math.toRadians(-90));
    final Pose2d BR_CENTER_PURPLE_BACKUP = new Pose2d(new Vector2d(-27,22), Math.toRadians(-90));
    final Pose2d BR_CENTER_MOVE_FROM_PIXEL = new Pose2d(new Vector2d(-33,18.5), Math.toRadians(-90));
    final Pose2d BR_CENTER_YELLOW_PREP_1 = new Pose2d(new Vector2d(-25.76,21), Math.toRadians(-90));
    final Pose2d BR_RIGHT_PROP_PUSH = new Pose2d(new Vector2d(-13,60), Math.toRadians(-90));
    final Pose2d BR_RIGHT_PURPLE_BACKUP = new Pose2d(new Vector2d(-34.5,27.5), Math.toRadians(-90));
    final Pose2d BR_RIGHT_MOVE_FROM_PIXEL = new Pose2d(new Vector2d(-38,27.5), Math.toRadians(-90));
    final Pose2d BR_RIGHT_YELLOW_PREP_1 = new Pose2d(new Vector2d(-25.76,27.6), Math.toRadians(-90));
    final Pose2d BR_LEFT_PROP_PUSH = new Pose2d(new Vector2d(-24,60), Math.toRadians(-90));
    final Pose2d BR_LEFT_PURPLE_BACKUP = new Pose2d(new Vector2d(-10.25,32.4), Math.toRadians(-90));
    final Pose2d BR_LEFT_MOVE_FROM_PIXEL = new Pose2d(new Vector2d(-24,31), Math.toRadians(-90));
    final Pose2d BR_LEFT_YELLOW_PREP_1 = new Pose2d(new Vector2d(-25.76,32), Math.toRadians(-90));
    final Pose2d BR_YELLOW_PREP_2 = new Pose2d(new Vector2d(-25.26,4), Math.toRadians(0));
    final Pose2d BR_YELLOW_PREP_3 = new Pose2d(new Vector2d(52.8,4), Math.toRadians(0));
    final Pose2d BR_READ_YELLOW_LEFT = new Pose2d(new Vector2d(54,41), Math.toRadians(0));
    final Pose2d BR_READ_YELLOW_CENTER = new Pose2d(new Vector2d(54,36), Math.toRadians(0));
    final Pose2d BR_READ_YELLOW_RIGHT = new Pose2d(new Vector2d(54,29.5), Math.toRadians(0));
    final Pose2d BR_PARK_BACKUP = new Pose2d(new Vector2d(45,30), Math.toRadians(0));
    final Pose2d BR_PARK_FINAL = new Pose2d(new Vector2d(50,58), Math.toRadians(0));



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
        fluffy.drive.pose = BR_START;
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
                        .strafeTo(BR_LEFT_PROP_PUSH.position)
                        .strafeTo(BR_LEFT_PURPLE_BACKUP.position)
                        .build());
        fluffy.deliverPurple();
        fluffy.retractPurple();
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo(BR_LEFT_MOVE_FROM_PIXEL.position)
                        .build());
    }
    public void deliverPurpleCenter() {
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo(BR_CENTER_PROP_PUSH.position)
                        .strafeTo(BR_CENTER_PURPLE_BACKUP.position)
                        .build());
        fluffy.deliverPurple();
        fluffy.retractPurple();
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo(BR_CENTER_MOVE_FROM_PIXEL.position)
                        .build());
    }
    public void deliverPurpleRight(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo(BR_RIGHT_PROP_PUSH.position)
                        //.strafeTo( new Vector2d(35,30))
                        .strafeTo(BR_RIGHT_PURPLE_BACKUP.position)
                        //.strafeTo( new Vector2d(25,26))
                        .build());
        fluffy.deliverPurple();
        fluffy.retractPurple();
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo(BR_RIGHT_MOVE_FROM_PIXEL.position)
                        .build());
    }

    public void yellowLeft(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo(BR_LEFT_YELLOW_PREP_1.position)
                        .strafeToLinearHeading(BR_YELLOW_PREP_2.position,BR_YELLOW_PREP_2.heading)
                        //.strafeToLinearHeading(new Vector2d(56, 13.76), Math.toRadians(-89.9))
                        .strafeTo(BR_YELLOW_PREP_3.position)
                        .strafeTo(BR_READ_YELLOW_LEFT.position)
                        .build());
    }
    public void yellowCenter(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo(BR_CENTER_YELLOW_PREP_1.position)
                        .strafeToLinearHeading(BR_YELLOW_PREP_2.position,BR_YELLOW_PREP_2.heading)
                        //.strafeToLinearHeading(new Vector2d(56, 13.76), Math.toRadians(-89.9))
                        .strafeTo(BR_YELLOW_PREP_3.position)
                        .strafeTo(BR_READ_YELLOW_CENTER.position)
                        .build());
    }
    public void yellowRight(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo(BR_RIGHT_YELLOW_PREP_1.position)
                        .strafeToLinearHeading(BR_YELLOW_PREP_2.position,BR_YELLOW_PREP_2.heading)
                        //.strafeToLinearHeading(new Vector2d(56, 13.76), Math.toRadians(-89.9))
                        .strafeTo(BR_YELLOW_PREP_3.position)
                        .strafeTo(BR_READ_YELLOW_RIGHT.position)
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
                        .strafeTo(BR_PARK_BACKUP.position)
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
                        .strafeTo(BR_PARK_FINAL.position)
                        .build());
    }


}

