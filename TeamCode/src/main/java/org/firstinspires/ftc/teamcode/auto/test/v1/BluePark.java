package org.firstinspires.ftc.teamcode.auto.test.v1;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "\uD83D\uDD35 - BluePark", group = "RoadRunner 1.0")
@Disabled
public class BluePark extends LinearOpMode {


    // Start position red near
    Pose2d RED_PARK_START_POSE = new Pose2d(-12, 60, -Math.PI/2.0);


    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, RED_PARK_START_POSE);
        TrajectoryActionBuilder traj = drive.actionBuilder(RED_PARK_START_POSE)
                .strafeToLinearHeading(new Vector2d(-12, 36), -Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-60, 60), -Math.toRadians(90));

        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(traj.build());

    } // runOpMode



}