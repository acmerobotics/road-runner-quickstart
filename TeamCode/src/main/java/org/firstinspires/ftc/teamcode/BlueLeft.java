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
@Autonomous(name = "BlueLeft", group = "robot", preselectTeleOp = "FluffyTeleOp")
public class BlueLeft extends LinearOpMode {
    //BRING GRABBER UP FIRST THING//
    AutoFluffy fluffy;
    String PATH;
    String SIDE = "Left";
    static double DELTA = 1;
    List<Recognition> currentRecognitions;

    final Pose2d BL_START = new Pose2d(new Vector2d(16.2, 63.2), Math.toRadians(-90));
    final Pose2d BL_CENTER_PROP_PUSH = new Pose2d(new Vector2d(16.2, 28), Math.toRadians(0));
    final Pose2d BL_CENTER_PURPLE_BACKUP = new Pose2d(new Vector2d(16.2, 33), Math.toRadians(0));
    final Pose2d BL_CENTER_YELLOW_PREP = new Pose2d(new Vector2d(16.7, 36.7), Math.toRadians(0));
    final Pose2d BL_READ_YELLOW_CENTER = new Pose2d(new Vector2d(40.2, 36.2), Math.toRadians(0));
    final Pose2d BL_LEFT_PROP_PUSH = new Pose2d(new Vector2d(30, 38.2), Math.toRadians(0));
    final Pose2d BL_LEFT_YELLOW_PREP = new Pose2d(new Vector2d(30.8, 45.2), Math.toRadians(0));
    final Pose2d BL_READ_YELLOW_LEFT = new Pose2d(new Vector2d(40.2, 40.8), Math.toRadians(0));
    final Pose2d BL_RIGHT_PROP_PUSH = new Pose2d(new Vector2d(21.2, 42.7), Math.toRadians(-90));
    final Pose2d BL_RIGHT_PURPLE_BACKUP = new Pose2d(new Vector2d(13, 30.7), Math.toRadians(-90));
    final Pose2d BL_RIGHT_MOVE_FROM_PURPLE = new Pose2d(new Vector2d(21.7,31.7), Math.toRadians(-90));
    final Pose2d BL_READ_YELLOW_RIGHT = new Pose2d(new Vector2d(40.2, 28.8), Math.toRadians(0));
    final Pose2d BL_PARK_BACKUP = new Pose2d(new Vector2d(37.2, 35.9), Math.toRadians(0));
    final Pose2d BL_PARK_FINAL_RIGHT = new Pose2d(new Vector2d(50.2, 12), Math.toRadians(0));
    final Pose2d BL_PARK_FINAL_CENTER = new Pose2d(new Vector2d(46.7, 36), Math.toRadians(0));
    final Pose2d BL_PARK_FINAL_LEFT = new Pose2d(new Vector2d(50.2, 60.2), Math.toRadians(0));

    Menu initMenu = new Menu(this);

    public void runOpMode() {
        initialize();

        initMenu.add(new MenuItem(3, "Park Loc (L=1 C=2 R=3)", 3, 1, 1));
        initMenu.add(new MenuItem(2, "Pixel Pos (L=1 R=2)", 2, 1, 1));
        initMenu.add(new MenuItem(0, "Wait Time (0-12)", 12, 0, 1));

        while (!isStarted() && !isStopRequested()) {
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
            PATH = fluffy.getPropLocation();
            initMenu.update();
            initMenu.display();
            telemetry.addData("Prop Location", PATH);
            telemetry.addData("Left Sat. Value", fluffy.getLeftMean());
            telemetry.addData("Center Sat. Value", fluffy.getCenterMean());
            telemetry.addData("Right Sat. Value", fluffy.getRightMean());

            telemetry.update();


        }

        sleep((long) initMenu.get(2) * 1000);
        // JRC: Turn off redFinder at this point.

        fluffy.drive.pose = BL_START;
        if (PATH.equals("Left")) {
            deliverPurpleLeft();
            yellowLeft();
        } else if (PATH.equals("Right")) {
            deliverPurpleRight();
            yellowRight();
        } else {
            deliverPurpleCenter();
            yellowCenter();
        }

        deliverYellow();
        park();
        RobotLog.i(String.format("Final position %f,%f", fluffy.drive.pose.position.x, fluffy.drive.pose.position.y));
    }


    public void initialize() {
        fluffy = new AutoFluffy(this, "Blue");
        fluffy.raiseGrabber();
    }

    public void deliverPurpleLeft() {
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(BL_LEFT_PROP_PUSH.position, BL_LEFT_PROP_PUSH.heading)
                        .build());
        fluffy.deliverPurple();

    }

    public void deliverPurpleCenter() {
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(BL_CENTER_PROP_PUSH.position, BL_CENTER_PROP_PUSH.heading)
                        .strafeTo(BL_CENTER_PURPLE_BACKUP.position)
                        .build());
        fluffy.deliverPurple();
    }

    public void deliverPurpleRight() {
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo(BL_RIGHT_PROP_PUSH.position)
                        .strafeTo(BL_RIGHT_PURPLE_BACKUP.position)
                        .build());
        fluffy.deliverPurple();
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo(BL_RIGHT_MOVE_FROM_PURPLE.position)
                        .build());
    }

    public void yellowLeft() {
        /* JRC: To read position, use
         * Don't forget to handle the null case gracefully!
         */
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo(BL_LEFT_YELLOW_PREP.position)
                        .strafeTo(BL_READ_YELLOW_LEFT.position)
                        .build());
        fluffy.retractPurple();
    }

    public void yellowCenter() {
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo(BL_CENTER_YELLOW_PREP.position)
                        .strafeTo(BL_READ_YELLOW_CENTER.position)
                        .build());
        fluffy.retractPurple();
    }

    public void yellowRight() {
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(BL_READ_YELLOW_RIGHT.position, BL_READ_YELLOW_RIGHT.heading)
                        .build());
        fluffy.retractPurple();
    }


    /*public void deliverYellow(){
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
        /*Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                       // .setReversed(true)
                        .strafeToLinearHeading(new Vector2d(fluffy.drive.pose.position.x, (fluffy.drive.pose.position.y + DELTA)),
                                Math.toRadians(89.9))
                       // .setReversed(false)
                        .build());*/
    //fluffy.lowerLift();

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

    public void deliverYellow() {
        Vector2d destination; //need to fix (offset)
        sleep(1000);
        fluffy.drive.pose = fluffy.getPoseFromAprilTag();
        if (PATH.equals("Left")) {
            destination = fluffy.tagPositions[0].plus(fluffy.DELIVERY_OFFSET_BLUE);
        } else if (PATH.equals("Center")) {
            destination = fluffy.tagPositions[1].plus(fluffy.DELIVERY_OFFSET_BLUE);
        } else {
            destination = fluffy.tagPositions[2].plus(fluffy.DELIVERY_OFFSET_BLUE);
        }
        if ((int) initMenu.get(1) == 1) {
            destination = destination.plus(new Vector2d(0, 3.5));

        }

        fluffy.raiseLift();

        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo(destination)
                        //.turnTo(fluffy.drive.pose.heading)
                        .build());

        fluffy.raiseFinger();

        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .lineToX(fluffy.drive.pose.position.x - 1)
                        .build());

    }


    public void park(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .lineToX(BL_PARK_BACKUP.position.x)
                        .build());
        fluffy.lowerLift();
        if ((int)initMenu.get(0) == 1){
            Actions.runBlocking(
                    fluffy.drive.actionBuilder(fluffy.drive.pose)
                            .strafeTo(BL_PARK_FINAL_LEFT.position)
                            .build());
        }
        else if ((int)initMenu.get(0) == 2){
            Actions.runBlocking(
                    fluffy.drive.actionBuilder(fluffy.drive.pose)
                            .strafeTo(BL_PARK_FINAL_CENTER.position)
                            .build());
        }
        else {
            Actions.runBlocking(
                    fluffy.drive.actionBuilder(fluffy.drive.pose)
                            .strafeTo(BL_PARK_FINAL_RIGHT.position)
                            .build());
        }



    }

}


    /*public void park(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .lineToY(fluffy.drive.pose.position.y - 5)
                        .build());
        fluffy.lowerLift();
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(new Vector2d(3 , 34), Math.toRadians(89.9))
                        .build());
    }*/

