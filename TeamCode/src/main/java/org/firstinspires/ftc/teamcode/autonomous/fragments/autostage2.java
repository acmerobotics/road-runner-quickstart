package org.firstinspires.ftc.teamcode.autonomous.fragments;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@Config
@Autonomous(name = "pick to deposit", group = "Auto Fragments")
public class autostage2 extends LinearOpMode {
    Pose2d startPose;
    MecanumDrive drive;


    @Override
    public void runOpMode() throws InterruptedException {
        startPose = new Pose2d(14, -61, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, startPose);


        TrajectoryActionBuilder build = drive.actionBuilder(startPose).strafeTo(new Vector2d(10, -34))
                .splineToConstantHeading(new Vector2d(-34,-35), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(new Vector2d(-36, -26), Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(new Vector2d(-47, -47), Math.toRadians(225)), Math.toRadians(180))
                ;
        waitForStart();
        Actions.runBlocking(new SequentialAction(build.build()));

    }
}

