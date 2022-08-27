package org.firstinspires.ftc.teamcode.drive.TestPaths;

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
public class Playground extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // declaring start pos
        Pose2d startPose = new Pose2d(0,0,Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        // building trajectories before starting program so it has enough time to do math
        Trajectory traj = drive.trajectoryBuilder(startPose, false)
                .strafeTo(new Vector2d(0,35))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj.end(), false)
                .lineToConstantHeading(new Vector2d(20,10))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), false)
                .splineToSplineHeading(new Pose2d(0,0, Math.toRadians(90)), Math.toRadians(180))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        // trajectories path order
        drive.followTrajectory(traj);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);


        // telemetry
    }
}
