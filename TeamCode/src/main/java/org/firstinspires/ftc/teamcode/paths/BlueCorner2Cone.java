package org.firstinspires.ftc.teamcode.paths;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "BlueCorner2Cone", group = "Auto")
public class BlueCorner2Cone extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        final SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
        final double radian90 = Math.toRadians(90);
        final Pose2d initialPose = new Pose2d(-36, 60, radian90);


        drivetrain.setPoseEstimate(initialPose);
        //Robot goes up to medium junction from the initial starting position at the blue corner
        final TrajectorySequence driveUpToJunction = drivetrain.trajectorySequenceBuilder(initialPose)
                .lineToLinearHeading(new Pose2d(11, -63))
                .lineToLinearHeading(new Pose2d(15, -15))
                .turn(Math.toRadians(-45))

                .build();



        //Robot drives to depot from high junction
        final TrajectorySequence driveToDepot = drivetrain.trajectorySequenceBuilder(driveUpToJunction.end())
                .lineToLinearHeading(new Pose2d(58, -12, 0))
                .build();
        //Robot drives back to high junction from depot
        final TrajectorySequence driveBackToJunction = drivetrain.trajectorySequenceBuilder(driveToDepot.end())
                .lineToLinearHeading(new Pose2d(33, -9, Math.toRadians(-45)))
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
