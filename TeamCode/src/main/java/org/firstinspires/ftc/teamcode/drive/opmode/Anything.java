package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
@Config
@Autonomous(group = "drive")
public class Anything extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();

//        Path path = new PathBuilder(new Pose2d(0,0,0))
//                .splineTo(new Pose2d(new vector2d(30,60),0))
//                .lineTo(new Vector2d(15,15,0))
//                .build();

//        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
//                .splineTo(new Vector2d(0, 40), -90)
//                .build();
//
//        drive.followTrajectory(traj);
//
//        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
//                .splineTo(new Vector2d(40, 0), 90)
//                .build();
//
//        drive.followTrajectory(traj1);
//
//        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d())
//                .splineTo(new Vector2d(0, -40), 90)
//                .build();
//
//        drive.followTrajectory(traj2);
//
//        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d())
//                .splineTo(new Vector2d(-40, 0), 0)
//                .build();

        //drive.followTrajectory(traj3);
        double X = 0;
        double Y = 0;
        double endTang = 0;
        while(opModeIsActive()) {
            X += 5;
            Y += 10;
            endTang += 3;
//            Trajectory traj = drive.trajectoryBuilder(new Pose2d())
//                    .splineTo(new Vector2d(X, Y),
//                            endTang).build();
//            drive.followTrajectory(traj);
            telemetry.addData("X, ", X);
            telemetry.addData("X, ", Y);
            telemetry.addData("X, ", endTang);
            telemetry.update();
        }

    }
}