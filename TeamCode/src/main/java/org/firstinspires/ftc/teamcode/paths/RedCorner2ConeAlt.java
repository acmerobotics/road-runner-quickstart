package org.firstinspires.ftc.teamcode.paths;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lift.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;

@Autonomous(name = "RedCorner2ConeAlt", group = "Auto")
public class RedCorner2ConeAlt extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        final SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
        final double radian90 = Math.toRadians(-90);
        final Pose2d initialPose = new Pose2d(-36, -63, radian90);


        drivetrain.setPoseEstimate(initialPose);
        //Robot goes up to high junction from the initial starting position at the blue corner
        final TrajectorySequence driveUpToJunction = drivetrain.trajectorySequenceBuilder(initialPose)
                .lineToConstantHeading(new Vector2d(-34, -31))
                .turn(Math.toRadians(-45))
                .build();
        //Robot drives to depot from high junction
        final TrajectorySequence driveToDepot = drivetrain.trajectorySequenceBuilder(driveUpToJunction.end())
                .lineToConstantHeading(new Vector2d(-36, -33))
                .turn(Math.toRadians(45))
                .lineToLinearHeading(new Pose2d(-36, -22, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-58, -12, Math.toRadians(180)))
                .build();
        //Robot drives back to high junction from depot
        final TrajectorySequence driveBackToJunction = drivetrain.trajectorySequenceBuilder(driveToDepot.end())
                .lineToConstantHeading(new Vector2d(-56, -12))
                .turn(Math.toRadians(-180))
                .lineToConstantHeading(new Vector2d(-35, -16))
                .turn(Math.toRadians(-45))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        drivetrain.followTrajectorySequence(driveUpToJunction);
        sleep(2000);
        drivetrain.followTrajectorySequence(driveToDepot);
        sleep(2000);
        drivetrain.followTrajectorySequence(driveBackToJunction);
    }

}
