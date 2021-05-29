package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/*
 * This is a straight line autonomous that will drop 1 wobble goal at square A on both sides of the field
 */

@Autonomous(group = "drive")
public class Auton_WallShoot extends LinearOpMode {
    //We have an issue with using the same auton for both sides. The start positions are different, and that could lead to potential issues.
    private Servo wobbleDropper;
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        long waitOffset = 10000;
        waitForStart();

        if (isStopRequested()) return;
        drive.moveTo("Away");
        sleep(waitOffset);
        drive.shootRings(3);
        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
//        Pose2d startPose = new Pose2d(-60, 48, Math.toRadians(0));

//        drive.setPoseEstimate(startPose);
//
//        Trajectory traj1 = drive.trajectoryBuilder(startPose)
//                .lineTo(new Vector2d(18,48))
//                .build();

//        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
//                .lineToConstantHeading(new Vector2d(-18, 48))
//                .build();

//        drive.followTrajectory(traj1);
//        drive.wobbleDrop();
//        sleep(2000);
//        drive.followTrajectory(traj2);
//        sleep(25000);
//        drive.followTrajectory(traj1);

//        wobbleDropper.setPosition(1);
//        drive.followTrajectory(
//                drive.trajectoryBuilder(traj.end(), true)
//                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
//                        .build()
//        );
    }
}