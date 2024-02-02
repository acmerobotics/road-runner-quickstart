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

    final Pose2d RL_START = new Pose2d(new Vector2d(-35, -65.5), Math.toRadians(90));
    final Pose2d RL_CENTER_PROP_PUSH = new Pose2d(new Vector2d(-50, -45.5), Math.toRadians(90));
    final Pose2d RL_CENTER_PURPLE_BACKUP = new Pose2d(new Vector2d(-50, -25.5), Math.toRadians(90));
    final Pose2d RL_CENTER_MOVE_FROM_PIXEL = new Pose2d(new Vector2d(-57, -24), Math.toRadians(90));
    final Pose2d RL_CENTER_YELLOW_PREP_1 = new Pose2d(new Vector2d(-57, -18.26), Math.toRadians(90));
    final Pose2d RL_RIGHT_PROP_PUSH = new Pose2d(new Vector2d(-41, -60.5), Math.toRadians(90));
    final Pose2d RL_RIGHT_PURPLE_BACKUP = new Pose2d(new Vector2d(-32.5, -31.9), Math.toRadians(90));
    final Pose2d RL_RIGHT_MOVE_FROM_PIXEL = new Pose2d(new Vector2d(-39, -35.5), Math.toRadians(90));
    final Pose2d RL_LEFT_PROP_PUSH_1 = new Pose2d(new Vector2d(-60, -60), Math.toRadians(90));
    final Pose2d RL_LEFT_PROP_PUSH_2 = new Pose2d(new Vector2d(-60, -31.5), Math.toRadians(90));
    final Pose2d RL_LEFT_PURPLE_BACKUP_1 = new Pose2d(new Vector2d(-45,-31.5), Math.toRadians(90));
    final Pose2d RL_LEFT_PURPLE_BACKUP_2 = new Pose2d(new Vector2d(-55.5,-31.5), Math.toRadians(90));
    final Pose2d RL_LEFT_MOVE_FROM_PIXEL = new Pose2d(new Vector2d(-60, -31.5), Math.toRadians(90));
    final Pose2d RL_LEFT_YELLOW_PREP_1 = new Pose2d(new Vector2d(-60, -20.26), Math.toRadians(90));
    final Pose2d RL_YELLOW_PREP_2 = new Pose2d(new Vector2d(-48.76, -14.5), Math.toRadians(0.1));
    final Pose2d RL_YELLOW_PREP_3 = new Pose2d(new Vector2d(35.8, -9.5), Math.toRadians(0.1));
    final Pose2d RL_READ_YELLOW_LEFT = new Pose2d(new Vector2d(43, -29.5), Math.toRadians(0.1));
    final Pose2d RL_READ_YELLOW_CENTER = new Pose2d(new Vector2d(43, -37), Math.toRadians(0.1));
    final Pose2d RL_READ_YELLOW_RIGHT = new Pose2d(new Vector2d(43, -41.5), Math.toRadians(0.1));
    final Pose2d RL_PARK_BACKUP = new Pose2d(new Vector2d(39, -37), Math.toRadians(0.1));
    final Pose2d RL_PARK_FINAL_LEFT = new Pose2d(new Vector2d(50.2,-12), Math.toRadians(0.1));
    final Pose2d RL_PARK_FINAL_CENTER = new Pose2d(new Vector2d(46.7,-36), Math.toRadians(0.1));
    final Pose2d RL_PARK_FINAL_RIGHT = new Pose2d(new Vector2d(50.2,-60.2), Math.toRadians(0.1));

    Menu initMenu = new Menu(this);


    List<Recognition> currentRecognitions;

    public void runOpMode() {
        initialize();

        initMenu.add(new MenuItem(3, "Park Loc (L=1 C=2 R=3)", 3, 1, 1));
        initMenu.add(new MenuItem(2, "Pixel Pos (L=1 R=2)", 2,1,1));
        initMenu.add(new MenuItem(2, "Wait Time (0-12)", 12,0,1));

        while (!isStarted() && !isStopRequested()) {
            PATH = fluffy.getPropLocation();
            initMenu.update();
            initMenu.display();
            telemetry.addData("Prop Location", PATH);
            telemetry.addData("Left Sat. Value", fluffy.getLeftMean());
            telemetry.addData("Center Sat. Value", fluffy.getCenterMean());
            telemetry.addData("Right Sat. Value", fluffy.getRightMean());
            telemetry.update();
        }

        sleep((long)initMenu.get(2)*1000);

        // JRC: Turn off redFinder at this point.
        fluffy.drive.pose = RL_START;
        if (PATH == "Left") {
            deliverPurpleLeft();
            yellowLeft();
        } else if (PATH == "Right") {
            deliverPurpleRight();
            yellowRight();
        } else {
            deliverPurpleCenter();
            yellowCenter();
        }
        deliverYellow();
        park();
        //RobotLog.i(String.format("Final position %f,%f",fluffy.drive.pose.position.x, fluffy.drive.pose.position.y));
    }


    public void initialize() {
        fluffy = new AutoFluffy(this, "Red");
        fluffy.raiseGrabber();
    }

    public void deliverPurpleLeft() {
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo(RL_LEFT_PROP_PUSH_1.position)
                        //.strafeTo( new Vector2d(35,30))
                        .strafeTo(RL_LEFT_PROP_PUSH_2.position)
                        //.strafeTo( new Vector2d(25,26))
                        .strafeTo(RL_LEFT_PURPLE_BACKUP_1.position)
                        .strafeTo(RL_LEFT_PURPLE_BACKUP_2.position)
                        .build());
        fluffy.deliverPurple();
        fluffy.retractPurple();
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo(RL_LEFT_MOVE_FROM_PIXEL.position)
                        .build());

    }

    public void deliverPurpleCenter() {
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo(RL_CENTER_PROP_PUSH.position)
                        .strafeTo(RL_CENTER_PURPLE_BACKUP.position)
                        .build());
        fluffy.deliverPurple();
        fluffy.retractPurple();
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo(RL_CENTER_MOVE_FROM_PIXEL.position)
                        .build());
    }

    public void deliverPurpleRight() {
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo(RL_RIGHT_PROP_PUSH.position)
                        .strafeTo(RL_RIGHT_PURPLE_BACKUP.position)
                        .build());
        fluffy.deliverPurple();
        fluffy.retractPurple();
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo(RL_RIGHT_MOVE_FROM_PIXEL.position)
                        .build());
    }

    public void yellowLeft() {
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo(RL_LEFT_YELLOW_PREP_1.position)
                        .strafeToLinearHeading(RL_YELLOW_PREP_2.position, RL_YELLOW_PREP_2.heading)
                        //.strafeToLinearHeading(new Vector2d(56, 13.76), Math.toRadians(-89.9))
                        .strafeTo(RL_YELLOW_PREP_3.position)
                        .strafeTo(RL_READ_YELLOW_LEFT.position)
                        .build());
    }

    public void yellowCenter() {
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeTo(RL_CENTER_YELLOW_PREP_1.position)
                        .strafeToLinearHeading(RL_YELLOW_PREP_2.position, RL_YELLOW_PREP_2.heading)
                        //.strafeToLinearHeading(new Vector2d(56, 13.76), Math.toRadians(-89.9))
                        .strafeTo(RL_YELLOW_PREP_3.position)
                        .strafeTo(RL_READ_YELLOW_CENTER.position)
                        .build());
    }

    public void yellowRight() {
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(RL_YELLOW_PREP_2.position, RL_YELLOW_PREP_2.heading)
                        //.strafeToLinearHeading(new Vector2d(56, 13.76), Math.toRadians(-89.9))
                        .strafeTo(RL_YELLOW_PREP_3.position)
                        .strafeTo(RL_READ_YELLOW_RIGHT.position)
                        .build());
    }

    public void yellowDriveUp() {
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .strafeToLinearHeading(new Vector2d(56, 13.76), Math.toRadians(0))
                        .turnTo(Math.toRadians(-89.9))
                        //.strafeToLinearHeading(new Vector2d(56, 13.76), Math.toRadians(-89.9))
                        .lineToY(-70.8)
                        .strafeToLinearHeading(new Vector2d(24.38, -78), Math.toRadians(-89.9))
                        .build());
    }



    public void deliverYellow() {
        Vector2d destination; //need to fix (offset)
        sleep(1000);
        fluffy.drive.pose = fluffy.getPoseFromAprilTag();
        if (PATH.equals("Left")) {
            destination = fluffy.tagPositions[3].plus(fluffy.DELIVERY_OFFSET_RED);
        } else if (PATH.equals("Center")) {
            destination = fluffy.tagPositions[4].plus(fluffy.DELIVERY_OFFSET_RED);
        } else {
            destination = fluffy.tagPositions[5].plus(fluffy.DELIVERY_OFFSET_RED);
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
    public void park(){
        Actions.runBlocking(
                fluffy.drive.actionBuilder(fluffy.drive.pose)
                        .lineToX(RL_PARK_BACKUP.position.x)
                        .build());
        fluffy.lowerLift();
        if ((int)initMenu.get(0) == 1){
            Actions.runBlocking(
                    fluffy.drive.actionBuilder(fluffy.drive.pose)
                            .strafeTo(RL_PARK_FINAL_LEFT.position)
                            .build());
        }
        else if ((int)initMenu.get(0) == 2){
            Actions.runBlocking(
                    fluffy.drive.actionBuilder(fluffy.drive.pose)
                            .strafeTo(RL_PARK_FINAL_CENTER.position)
                            .build());
        }
        else {
            Actions.runBlocking(
                    fluffy.drive.actionBuilder(fluffy.drive.pose)
                            .strafeTo(RL_PARK_FINAL_RIGHT.position)
                            .build());
        }

    }


}
