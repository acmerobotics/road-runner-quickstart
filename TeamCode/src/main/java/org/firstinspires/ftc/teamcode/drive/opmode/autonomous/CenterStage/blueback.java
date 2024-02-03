package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.CenterStage;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Blue Back auto")
@Config
public class blueback extends LinearOpMode {

    private final Pose2d blueBackStart = new Pose2d(-36,60, Math.toRadians(-90));
    private final Pose2d blueBackEnd = new Pose2d(48, 36, Math.toRadians(0));


    // drive forward 2 feet
    private final Pose2d centerOfLines = new Pose2d(-36, 36, Math.toRadians(-90));


    // the poses for the lines on the ground




    private final Pose2d rightLine = new Pose2d(-48, 30, Math.toRadians(0));
    private final Pose2d  leftLine = new Pose2d(-24, 30, Math.toRadians(0));
    private final Pose2d centerLine = new Pose2d(-36, 24.5, Math.toRadians(-90));

    // truss
    private final Pose2d blueBackTruss = new Pose2d(-24, 36, Math.toRadians(0));

    private Pose2d _pose;


    SampleMecanumDrive drive;


    @Override
    public void runOpMode() throws InterruptedException {
        drive.initArm();


        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(blueBackStart);
        TrajectorySequence placePurpleBack = drive.trajectorySequenceBuilder(_pose)
                .lineToSplineHeading(blueBackTruss)
                .lineToSplineHeading(blueBackEnd)
                .build();
        drive.followTrajectorySequence(placePurpleBack);

    }
}
