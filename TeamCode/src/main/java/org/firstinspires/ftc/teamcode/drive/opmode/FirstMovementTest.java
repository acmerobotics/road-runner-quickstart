package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

/*
 * This is an example of a more complex path to really test the tuning.
 */


@Autonomous(group = "drive")
public class FirstMovementTest extends LinearOpMode {
    private Servo wobbleDropper;
    private double slowerVelocity = 3;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        wobbleDropper = hardwareMap.get(Servo.class, "wobbleDropper");
        waitForStart();

        if (isStopRequested()) return;

        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(-60, 48, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-24, 48))
                .splineTo(new Vector2d(36, 19), Math.toRadians(0))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToConstantHeading(new Vector2d(-18, 19))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineTo(
                new Vector2d(-27, 19),
                SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineTo(new Vector2d(12, 19))
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end().plus(new Pose2d(0, 0, Math.toRadians(-135))))
                .forward(2)
                .build();

        drive.followTrajectory(traj1);
        drive.wobbleDrop();
        sleep(2000);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        sleep(2000);
        drive.followTrajectory(traj4);
        drive.turn(Math.toRadians(-135));
        sleep(2000);
        drive.followTrajectory(traj5);

//        wobbleDropper.setPosition(1);
//        drive.followTrajectory(
//                drive.trajectoryBuilder(traj.end(), true)
//                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
//                        .build()
//        );
    }
}
