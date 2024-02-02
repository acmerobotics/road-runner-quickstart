package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.CenterStage;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage.blueCameraPipeline.MovementDirection.LEFT;
import static org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage.blueCameraPipeline.MovementDirection.MIDDLE;
import static org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage.blueCameraPipeline.MovementDirection.RIGHT;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage.blueCameraPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="piped", group = "practice")
@Config
public class ryanTriplett extends LinearOpMode {
    private final Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));
    private final Pose2d endPose = new Pose2d(0, 15, Math.toRadians(0));

    private final Pose2d endPose2 = new Pose2d(15, 0, Math.toRadians(0));
    private final Pose2d endPose3 = new Pose2d(15, 0, Math.toRadians(0));

    private final double travelSpeed = 45.0, travelAccel = 30.0;

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    blueCameraPipeline.MovementDirection direction;
    @Override
    public void runOpMode() throws InterruptedException {


        while (!isStarted()) {
            drive.initArm();

            // Initialize the camera somewhere here.
            // Create an instance of the pipeline
            blueCameraPipeline pipeline = new blueCameraPipeline();

            // Run the getDirection function from the pipeline
            direction = pipeline.getDirection();
        }

        TrajectorySequence leftProp = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(endPose,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                ).build();

        TrajectorySequence middleProp = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(endPose2,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                ).build();

        TrajectorySequence rightProp = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(endPose3,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                ).build();

        if (direction == LEFT) {
            drive.followTrajectorySequence(leftProp);
        }
        else if (direction == MIDDLE) {
            drive.followTrajectorySequence(middleProp);
        }
        else if (direction == RIGHT) {
            drive.followTrajectorySequence(rightProp);
        }
    }
}
/*
MESSAGE TO JAXSON
This is just showing what I meant, whether we do it this way or how you're thinking, I don't care
I just can't seem to understand what you're saying we should do
 */