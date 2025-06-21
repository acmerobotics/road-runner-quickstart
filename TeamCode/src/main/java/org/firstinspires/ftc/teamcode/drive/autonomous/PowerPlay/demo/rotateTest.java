package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.PowerPlay.demo;/*
package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.PowerPlay.demo;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Config
@Autonomous(group = "demo")
public class rotateTest extends LinearOpMode {

    private final Pose2d startPose = new Pose2d(36, -63, Math.toRadians(90));
    private final Pose2d scorePose = new Pose2d(36, -12, Math.toRadians(90));
    private final Pose2d stackPose = new Pose2d(40, -10, Math.toRadians(5));

    private final double travelSpeed = 45.0, travelAccel = 30.0;

    private final Pose2d[] parkingSpots = {new Pose2d(12, -17, Math.toRadians(90)), new Pose2d(36,
            -20, Math.toRadians(90)), new Pose2d(60, -17, Math.toRadians(90))};

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        // Tell the robot where it is based on a pose created earlier
        drive.setPoseEstimate(startPose);

        // Create the first trajectory to be run when the round starts

        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(scorePose,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                .build();



        waitForStart();

        drive.followTrajectorySequence(goToStack);

        for (int i = 0; i < 3; i++) {
            sleep(500);

            drive.updatePoseEstimate();
            TrajectorySequence turnRight = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .turn(Math.toRadians(-90), Math.toRadians(30), Math.toRadians(30))
                    .build();

            drive.followTrajectorySequence(turnRight);

            sleep(500);

            drive.updatePoseEstimate();
            TrajectorySequence turnLeft = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .turn(Math.toRadians(90), Math.toRadians(30), Math.toRadians(30))
                    .build();

            drive.followTrajectorySequence(turnLeft);
        }
    }
}*/
