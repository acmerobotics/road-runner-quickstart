package org.firstinspires.ftc.teamcode.MainCode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import org.firstinspires.ftc.teamcode.MainCode.Autonomous.Constants.Spike;
import org.firstinspires.ftc.teamcode.MainCode.Autonomous.Constants.Alliance;
import org.firstinspires.ftc.teamcode.MainCode.Autonomous.Constants.Side;

@Config
@TeleOp(name="Autonomous", group="Linear Opmode")

public final class MainAuto extends LinearOpMode {
    public static Side start;
    public static Spike lcr;
    public static Alliance color;

    //for dashboard
    public static String startValue = "";
    public static String lcrValue = "";
    public static String colorValue = "";


    public void runOpMode() throws InterruptedException {
        Pose2d startingPose;
        Pose2d nextPose;
        int pixelPusherX = 0; //3
        int pixelPusherY = 0; //-10
        MecanumDrive drive;
        double reflect;
        ConfigDashboard();
//TODO on blue side, flip left and right as they are mirrored and wrong values
        waitForStart();
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class) && start.equals(Side.BACKSTAGE)) { //Backstage
            if (color.equals(Alliance.RED)) {
                reflect = 1.0;
            } else{
                reflect = -1.0;
            }
                startingPose = new Pose2d(12, -64*reflect, Math.toRadians(90*reflect));
                drive = new MecanumDrive(hardwareMap, startingPose);
                switch (lcr) {
                    case LEFT:
                        nextPose = new Pose2d(2 + pixelPusherX, -30*reflect + pixelPusherY, Math.toRadians(90*reflect));
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(nextPose.position, nextPose.heading)
                                        .build());
                        break;
                    case CENTER:
                        nextPose = new Pose2d(12 + pixelPusherX, -26*reflect + pixelPusherY, Math.toRadians(90*reflect));
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(nextPose.position, nextPose.heading)
                                        .build());
                        break;
                    case RIGHT:
                        nextPose = new Pose2d(22 + pixelPusherX, -30*reflect + pixelPusherY, Math.toRadians(90*reflect));
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(nextPose.position, nextPose.heading)
                                        .build());
                        break;
                }
            } else if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class) && start.equals(Side.AUDIENCE)) { //AudienceRed
                startingPose = new Pose2d(-36, -64, Math.PI / 2);
                drive = new MecanumDrive(hardwareMap, startingPose);
                switch (lcr) {
                    case LEFT:
                        nextPose = new Pose2d(-46 + pixelPusherX, -30 + pixelPusherY, Math.PI / 2);
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(nextPose.position, nextPose.heading)
                                        .build());
                        break;
                    case CENTER:
                        nextPose = new Pose2d(-36 + pixelPusherX, -26 + pixelPusherY, Math.PI / 2);
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(nextPose.position, nextPose.heading)
                                        .build());
                        break;
                    case RIGHT:
                        nextPose = new Pose2d(-26 + pixelPusherX, -30 + pixelPusherY, Math.PI / 2);
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(nextPose.position, nextPose.heading)
                                        .build());
                        break;

                }
            } else if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class) && start.equals("BB")) { //BackstageBlue
                startingPose = new Pose2d(12, 64, Math.PI * 3 / 2);
                drive = new MecanumDrive(hardwareMap, startingPose);
                switch (lcr) {
                    case LEFT:
                        nextPose = new Pose2d(22 + pixelPusherX, 30 + pixelPusherY, Math.PI * 3 / 2);
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(nextPose.position, nextPose.heading)
                                        .build());
                        break;
                    case CENTER:
                        nextPose = new Pose2d(12 + pixelPusherX, 26 + pixelPusherY, Math.PI * 3 / 2);
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(nextPose.position, nextPose.heading)
                                        .build());
                        break;
                    case RIGHT:
                        nextPose = new Pose2d(2 + pixelPusherX, 30 + pixelPusherY, Math.PI * 3 / 2);
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(nextPose.position, nextPose.heading)
                                        .build());
                        break;

                }
            } else //(TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class) && start.equals("AB"))
            { //AudienceBlue
                startingPose = new Pose2d(-36, 64, Math.PI * 3 / 2);
                drive = new MecanumDrive(hardwareMap, startingPose);
                switch (lcr) {
                    case LEFT:
                        nextPose = new Pose2d(-26 + pixelPusherX, 30 + pixelPusherY, Math.PI * 3 / 2);
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(nextPose.position, nextPose.heading)
                                        .build());
                        break;
                    case CENTER:
                        nextPose = new Pose2d(-36 + pixelPusherX, 26 + pixelPusherY, Math.PI * 3 / 2);
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(nextPose.position, nextPose.heading)
                                        .build());
                        break;
                    case RIGHT:
                        nextPose = new Pose2d(-46 + pixelPusherX, 30 + pixelPusherY, Math.PI * 3 / 2);
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(nextPose.position, nextPose.heading)
                                        .build());
                        break;

                }
            }
        /* if (start == "BR" || start == "AR"){
            nextPose = new Pose2d(drive.pose.position.x, startingPose.position.y + 6, Math.PI / 2);
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .splineTo(nextPose.position, nextPose.heading)
                            .build());
        }
        else {
            nextPose = new Pose2d(drive.pose.position.x, startingPose.position.y - 6, Math.PI*3/2);
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .splineTo(nextPose.position, nextPose.heading)
                            .build());
        } */


        }

    private static void ConfigDashboard() {
        switch (startValue){
            case "AUDIENCE":
                start = Side.AUDIENCE;
                break;
            case "BACKSTAGE":
                start = Side.BACKSTAGE;
                break;
        }
        switch (colorValue) {
            case "RED":
                color = Alliance.RED;
                break;
            case "BLUE":
                color = Alliance.BLUE;
                break;
        }
        switch (lcrValue) {
            case "LEFT":
                lcr = Spike.LEFT;
                break;
            case "CENTER":
                lcr = Spike.CENTER;
                break;
            case "RIGHT":
                lcr = Spike.RIGHT;
                break;
        }
    }
}



//Width of robot is 18 in, length is 16 in
//redBackdropSide_OpMode
// Starting Pose 12, -64, pi
//Left   Spike Vector2d(2, -30), Math.PI / 2
//Center Spike Vector2d(12, -26), Math.PI / 2
//Right  Spike Vector2d(22, -30), Math.PI / 2

//redAudienceSide_OpMode
// Starting Pose -36, -64, pi
//Left   Spike Vector2d(-46, -30), Math.PI / 2
//Center Spike Vector2d(-36, -26), Math.PI / 2
//Right  Spike Vector2d(-26, -30), Math.PI / 2

//blueBackdropSide_OpMode
// Starting Pose 12, 64, 2pi
//Left   Spike Vector2d(22, 30), Math.PI / 2
//Center Spike Vector2d(12, 26), Math.PI / 2
//Right  Spike Vector2d(2, 30), Math.PI / 2

//blueAudienceSide_OpMode
// Starting Pose -36, 64, 2pi
//Left   Spike Vector2d(-26, 30), Math.PI / 2
//Center Spike Vector2d(-36, -26), Math.PI / 2
//Right  Spike Vector2d(-46, 30), Math.PI / 2

//Detect spike

//If left go to left marker

//If center go to center marker

//If right go to right marker

//If in audience side wait a little bit for team to place on backdrop

//Start driving to backboard, if it detects an april tag on backboard it drives to it.

//Filter out all april tags except(left, center, right) and position robot to drop

//Raise linear slides

//Move claw and drop

//Normalize the robot(Get linear slides back down)

//If in backdrop side move right for team to place in backdrop