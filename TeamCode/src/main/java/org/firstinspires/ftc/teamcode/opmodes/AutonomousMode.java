package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@Config
@Autonomous(group = "drive")
public class AutonomousMode extends LinearOpMode {
    public static double DISTANCE = 50;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d())
                .forward(-5)
                .build();

        Trajectory trajectoryBackward = drive.trajectoryBuilder(trajectoryForward.end())
                .back(-5)
                .build();
        Trajectory trajectoryLeft = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(-5)
                .build();

        waitForStart();

        drive.followTrajectory(trajectoryForward);
        drive.followTrajectory(trajectoryBackward);
        drive.followTrajectory(trajectoryLeft);
        drive.followTrajectory(trajectoryForward);

    }
}