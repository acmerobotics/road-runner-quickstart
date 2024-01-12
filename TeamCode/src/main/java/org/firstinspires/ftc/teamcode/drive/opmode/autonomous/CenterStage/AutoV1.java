package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.CenterStage;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name = "lf auto")
@Config
public class AutoV1 extends LinearOpMode {

    //pivate stuff abject calling
    private final Pose2d StartLF = new Pose2d(0,0,Math.toRadians(90));
    private final Pose2d PurpPos = new Pose2d(10,0,Math.toRadians(90));
    SampleMecanumDrive drive;
    private final double travelSpeed = 45.0, travelAccel = 30.0;

    @Override
    public void runOpMode() throws InterruptedException {
        drive.initArm();

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(StartLF);
        TrajectorySequence PlacePurple = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(PurpPos,
                SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(travelAccel)
        )
                .build();

        drive.followTrajectorySequence(PlacePurple);
        drive.setGrip(false);

    }
}   
