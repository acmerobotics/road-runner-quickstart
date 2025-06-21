package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.CenterStage;/*
package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.CenterStage;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.teamcode.drive.opmode.visionPowerPlay.parkingZoneFinder;

@Autonomous(name = "N")
@Config
public class NandN extends LinearOpMode {

    private final Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));

    private final Pose2d endPose = new Pose2d(12, 12, Math.toRadians(0));

    private final double travelSpeed = 45.0, travelAccel = 30.0;

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        drive.initArm();

        while (!isStarted()) {

            drive.setBothGrip(true);
        }


        TrajectorySequence norm = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(endPose,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                .build();

        drive.setSlideVelocity(4000, drive.slideRight, drive.slideLeft, drive.wristMotor);

        drive.followTrajectorySequence(norm);

    }

}
*/
