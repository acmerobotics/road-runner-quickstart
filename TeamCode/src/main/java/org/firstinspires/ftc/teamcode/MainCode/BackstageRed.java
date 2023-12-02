package org.firstinspires.ftc.teamcode.MainCode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Config
@TeleOp(name="autonomousMain", group="Linear Opmode")

public final class BackstageRed extends LinearOpMode {
    public static String start = "BR";
    public static String lcr = "c";
    public void runOpMode() throws InterruptedException {
        Pose2d startingPose;
        Pose2d nextPose;
        int pixelPusherX = 3;
        int pixelPusherY = -10;
        MecanumDrive drive;

        waitForStart();
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class) && start.equals("BR")) { //BackstageRed
            startingPose = new Pose2d(12, -64, Math.PI/2);
            drive = new MecanumDrive(hardwareMap, startingPose);
            switch (lcr) {
                case "l":
                    nextPose = new Pose2d(2 + pixelPusherX, -30 + pixelPusherY, Math.PI / 2);
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineToConstantHeading(nextPose.position, nextPose.heading)
                                    .build());
                    break;
                case "c":
                    nextPose = new Pose2d(12 + pixelPusherX, -26 + pixelPusherY, Math.PI / 2);
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineToConstantHeading(nextPose.position, nextPose.heading)
                                    .build());
                    break;
                case "r":
                    nextPose = new Pose2d(22 + pixelPusherX, -30 + pixelPusherY, Math.PI / 2);
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineToConstantHeading(nextPose.position, nextPose.heading)
                                    .build());
                    break;

                }
            }
        else if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class) && start.equals("AR")) { //AudienceRed
            startingPose = new Pose2d(-36, -64, Math.PI/2);
            drive = new MecanumDrive(hardwareMap, startingPose);
            switch (lcr){
                    case "l":
                        nextPose = new Pose2d(-46 + pixelPusherX, -30 + pixelPusherY, Math.PI / 2);
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(nextPose.position, nextPose.heading)
                                        .build());
                        break;
                    case "c":
                        nextPose = new Pose2d(-36 + pixelPusherX, -26 + pixelPusherY, Math.PI / 2);
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(nextPose.position, nextPose.heading)
                                        .build());
                        break;
                    case "r":
                        nextPose = new Pose2d(-26 + pixelPusherX, -30 + pixelPusherY, Math.PI / 2);
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(nextPose.position, nextPose.heading)
                                        .build());
                        break;

                }
            }
        else if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class) && start.equals("BB")) { //BackstageBlue
            startingPose = new Pose2d(12, 64, Math.PI*3/2);
            drive = new MecanumDrive(hardwareMap, startingPose);
            switch (lcr){
                case "l":
                    nextPose = new Pose2d(22 + pixelPusherX, 30 + pixelPusherY, Math.PI*3 / 2);
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineToConstantHeading(nextPose.position, nextPose.heading)
                                    .build());
                    break;
                case "c":
                    nextPose = new Pose2d(12 + pixelPusherX, 26 + pixelPusherY, Math.PI*3 / 2);
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineToConstantHeading(nextPose.position, nextPose.heading)
                                    .build());
                    break;
                case "r":
                    nextPose = new Pose2d(2 + pixelPusherX, 30 + pixelPusherY, Math.PI*3 / 2);
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineToConstantHeading(nextPose.position, nextPose.heading)
                                    .build());
                    break;

            }
        }
        else //(TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class) && start.equals("AB"))
        { //AudienceBlue
            startingPose = new Pose2d(-36, 64, Math.PI*3/2);
            drive = new MecanumDrive(hardwareMap, startingPose);
            switch (lcr){
                case "l":
                    nextPose = new Pose2d(-26 + pixelPusherX, 30 + pixelPusherY, Math.PI*3 / 2);
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineToConstantHeading(nextPose.position, nextPose.heading)
                                    .build());
                    break;
                case "c":
                    nextPose = new Pose2d(-36 + pixelPusherX, 26 + pixelPusherY, Math.PI*3 / 2);
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineToConstantHeading(nextPose.position, nextPose.heading)
                                    .build());
                    break;
                case "r":
                    nextPose = new Pose2d(-46 + pixelPusherX, 30 + pixelPusherY, Math.PI*3 / 2);
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