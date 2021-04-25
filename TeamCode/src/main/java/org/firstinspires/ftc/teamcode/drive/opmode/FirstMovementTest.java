package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */


@Autonomous(group = "drive")
public class FirstMovementTest extends LinearOpMode {
    private Servo wobbleDropper;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        wobbleDropper = hardwareMap.get(Servo.class, "wobbleDropper");
        waitForStart();

        if (isStopRequested()) return;

        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(-60, 48, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory traj = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(36, 25), 0)
                .build();

        drive.followTrajectory(traj);

        sleep(2000);

        wobbleDropper.setPosition(1);
//        drive.followTrajectory(
//                drive.trajectoryBuilder(traj.end(), true)
//                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
//                        .build()
//        );
    }
}
