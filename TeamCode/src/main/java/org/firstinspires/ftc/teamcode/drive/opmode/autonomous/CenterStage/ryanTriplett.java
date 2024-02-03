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
import org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage.Camerainitialization;
import org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage.blueCameraPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="piped", group = "practice")
@Config
public class ryanTriplett extends LinearOpMode {
    private final Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));
    private final Pose2d endPose = new Pose2d(0, 15, Math.toRadians(0));

    private final Pose2d endPose2 = new Pose2d(15, 0, Math.toRadians(0));
    private final Pose2d endPose3 = new Pose2d(15, 0, Math.toRadians(0));

    private final double travelSpeed = 45.0, travelAccel = 30.0;

    SampleMecanumDrive drive;

    blueCameraPipeline ourCam = new blueCameraPipeline();
    private blueCameraPipeline.MovementDirection linePlace;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        while (!isStarted()) {
            drive.initArm();

            linePlace = ourCam.getDirection();

            telemetry.addData("location",ourCam.getDirection());
            telemetry.addData("location",linePlace);

            telemetry.update();
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


        if (linePlace == LEFT) {
            drive.followTrajectorySequence(leftProp);
        }
        else if (linePlace == MIDDLE) {
            drive.followTrajectorySequence(middleProp);
        }
        else if (linePlace == RIGHT) {
            drive.followTrajectorySequence(rightProp);
        }
    }
}