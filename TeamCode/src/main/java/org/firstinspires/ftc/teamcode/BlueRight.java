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

    final Pose2d BR_START = new Pose2d(new Vector2d(-35,65.5), Math.toRadians(-90));
    final Pose2d BR_CENTER_PROP_PUSH = new Pose2d(new Vector2d(-50,45.5), Math.toRadians(-90));
    final Pose2d BR_CENTER_PURPLE_BACKUP_1 = new Pose2d(new Vector2d(-50,25.5), Math.toRadians(-90));
    final Pose2d BR_CENTER_PURPLE_BACKUP_2 = new Pose2d(new Vector2d(-41,25.5), Math.toRadians(-90));
    final Pose2d BR_CENTER_MOVE_FROM_PIXEL = new Pose2d(new Vector2d(-50,24), Math.toRadians(-90));
    final Pose2d BR_CENTER_YELLOW_PREP_1 = new Pose2d(new Vector2d(-50,18.26), Math.toRadians(-90));
    final Pose2d BR_RIGHT_PROP_PUSH_1 = new Pose2d(new Vector2d(-54,57), Math.toRadians(-90));
    final Pose2d BR_RIGHT_PROP_PUSH_2 = new Pose2d(new Vector2d(-54,31.5), Math.toRadians(-90));
    final Pose2d BR_RIGHT_PURPLE_BACKUP_1 = new Pose2d(new Vector2d(-44,31.5), Math.toRadians(-90));
    final Pose2d BR_RIGHT_PURPLE_BACKUP_2 = new Pose2d(new Vector2d(-51,31.5), Math.toRadians(-90));
    final Pose2d BR_RIGHT_MOVE_FROM_PIXEL = new Pose2d(new Vector2d(-54,31.5), Math.toRadians(-90));
    final Pose2d BR_RIGHT_YELLOW_PREP_1 = new Pose2d(new Vector2d(-52,20.26), Math.toRadians(-90));
    final Pose2d BR_LEFT_PROP_PUSH = new Pose2d(new Vector2d(-41,60.5), Math.toRadians(-90));
    final Pose2d BR_LEFT_PURPLE_BACKUP = new Pose2d(new Vector2d(-26.5,31.9), Math.toRadians(-90));
    final Pose2d BR_LEFT_MOVE_FROM_PIXEL = new Pose2d(new Vector2d(-39,35.5), Math.toRadians(-90));
    final Pose2d BR_YELLOW_PREP_2 = new Pose2d(new Vector2d(-45.6,12.5), Math.toRadians(0));
    final Pose2d BR_YELLOW_PREP_3 = new Pose2d(new Vector2d(35.8,9.5), Math.toRadians(0));
    final Pose2d BR_READ_YELLOW_LEFT = new Pose2d(new Vector2d(43,41.5), Math.toRadians(0));
    final Pose2d BR_READ_YELLOW_CENTER = new Pose2d(new Vector2d(43,37), Math.toRadians(0));
    final Pose2d BR_READ_YELLOW_RIGHT = new Pose2d(new Vector2d(43,29.5), Math.toRadians(0));
    final Pose2d BR_PARK_BACKUP = new Pose2d(new Vector2d(39,37), Math.toRadians(0));
    final Pose2d BR_PARK_FINAL_LEFT = new Pose2d(new Vector2d(50.2,60.2), Math.toRadians(0));
    final Pose2d BR_PARK_FINAL_CENTER = new Pose2d(new Vector2d(46.7,36), Math.toRadians(0));
    final Pose2d BR_PARK_FINAL_RIGHT = new Pose2d(new Vector2d(50.2,12), Math.toRadians(0));

    Menu initMenu = new Menu(this);

    List<Recognition> currentRecognitions;
    public void runOpMode(){
        initialize();
        initMenu.add(new MenuItem(3, "Park Loc (L=1 C=2 R=3)", 3, 1, 1));
        initMenu.add(new MenuItem(2, "Pixel Pos (L=1 R=2)", 2,1,1));
        initMenu.add(new MenuItem(2, "Wait Time (0-12)", 12,0,1));
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
            initMenu.update();
            initMenu.display();
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
        deliverYellow();
        park();
        //RobotLog.i(String.format("Final position %f,%f",fluffy.drive.pose.position.x, fluffy.drive.pose.position.y));
    }


    public void initialize(){
        fluffy = new AutoFluffy(this, "Blue");
        fluffy.raiseGrabber();
    }


    public void deliverPurpleLeft() {
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
                        .strafeTo(BR_CENTER_PURPLE_BACKUP_1.position)
                        .strafeTo(BR_CENTER_PURPLE_BACKUP_2.position)
                        .build());
        fluffy.deliverPurple();
        fluffy.retractPurple();
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo(BR_CENTER_MOVE_FROM_PIXEL.position)
                        .build());
    }

    public void deliverPurpleRight() {
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo(BR_RIGHT_PROP_PUSH_1.position)
                        //.strafeTo( new Vector2d(35,30))
                        .strafeTo(BR_RIGHT_PROP_PUSH_2.position)
                        //.strafeTo( new Vector2d(25,26))
                        .strafeTo(BR_RIGHT_PURPLE_BACKUP_1.position)
                        .strafeTo(BR_RIGHT_PURPLE_BACKUP_2.position)
                        .build());
        fluffy.deliverPurple();
        fluffy.retractPurple();
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo(BR_RIGHT_MOVE_FROM_PIXEL.position)
                        .build());

    }

    public void yellowLeft() {
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(BR_YELLOW_PREP_2.position, BR_YELLOW_PREP_2.heading)
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

    public void deliverYellow() {
        Vector2d destination; //need to fix (offset)
        sleep(1000);
        fluffy.drive.pose = fluffy.getPoseFromAprilTag();
        if (PATH.equals("Left")) {
            destination = fluffy.tagPositions[0].plus(fluffy.DELIVERY_OFFSET);
        } else if (PATH.equals("Center")) {
            destination = fluffy.tagPositions[1].plus(fluffy.DELIVERY_OFFSET);
        } else {
            destination = fluffy.tagPositions[2].plus(fluffy.DELIVERY_OFFSET);
        }
        if ((int) initMenu.get(1) == 1) {
            destination = destination.plus(new Vector2d(0, 4.5));

        }

        fluffy.raiseLiftHigh();

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
    /*public void park(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo(BR_PARK_FINAL.position)
                        .build());

     */
    public void park(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .lineToX(BR_PARK_BACKUP.position.x)
                        .build());
        fluffy.lowerLift();
        if ((int)initMenu.get(0) == 1){
            Actions.runBlocking(
                    fluffy.drive.actionBuilder(fluffy.drive.pose)
                            .strafeTo(BR_PARK_FINAL_LEFT.position)
                            .build());
        }
        else if ((int)initMenu.get(0) == 2){
            Actions.runBlocking(
                    fluffy.drive.actionBuilder(fluffy.drive.pose)
                            .strafeTo(BR_PARK_FINAL_CENTER.position)
                            .build());
        }
        else {
            Actions.runBlocking(
                    fluffy.drive.actionBuilder(fluffy.drive.pose)
                            .strafeTo(BR_PARK_FINAL_RIGHT.position)
                            .build());
        }



    }



}

