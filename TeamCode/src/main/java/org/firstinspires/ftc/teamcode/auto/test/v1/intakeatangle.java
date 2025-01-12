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


@Autonomous(name = "intakeatangle-driveonly", group = "RoadRunner 1.0")
@Disabled
public class intakeatangle extends LinearOpMode {


    // Start position red near
    Pose2d RED_SCORE_START_POSE = new Pose2d(-38, -60, Math.toRadians(180));


    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, RED_SCORE_START_POSE);
        TrajectoryActionBuilder traj = drive.actionBuilder(RED_SCORE_START_POSE)
                .strafeToLinearHeading(new Vector2d(-38, -56), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-50, -50), Math.toRadians(180+45))
                .strafeToLinearHeading(new Vector2d(-28, -30), Math.toRadians(161));
        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(traj.build());

    } // runOpMode



}
