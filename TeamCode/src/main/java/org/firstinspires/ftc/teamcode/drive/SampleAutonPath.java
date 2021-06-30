package org.firstinspires.ftc.teamcode.drive.SampleAutonPaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SampleAutonPath extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-63, -48, 0);

        drive.setPoseEstimate(startPose);
        Trajectory traj = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-12, -36), 0)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
                .splineTo(new Vector2d(6 , -40),  Math.toRadians(90))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(-12 , -36), Math.toRadians(45))
                .splineTo(new Vector2d(-39 , -12),  Math.toRadians(0))
                .build();
        waitForStart();

        if (isStopRequested()) return;
        drive.followTrajectory(traj);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
    }
}
