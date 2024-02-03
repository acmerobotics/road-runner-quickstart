package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.CenterStage;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.net.ProxySelector;

@Autonomous(name = "redback auto")
@Config
public class redback extends LinearOpMode {

    private final Pose2d redBackStart = new Pose2d(-36,-60, Math.toRadians(90));
    private final Pose2d redBackEnd = new Pose2d(48, -36, Math.toRadians(0));


    // drive forward 2 feet
    private final Pose2d centerOfLines = new Pose2d(-36, 36, Math.toRadians(-90));


    // the poses for the lines on the ground




    private final Pose2d rightLine = new Pose2d(-24, -32, Math.toRadians(0));
    private final Pose2d  leftLine = new Pose2d(-48, -32, Math.toRadians(0));
    private final Pose2d centerLine = new Pose2d(-36, -24.5, Math.toRadians(-90));

    // truss
    private final Pose2d redBackTruss = new Pose2d(-24, -36, Math.toRadians(0));
    private final Pose2d redFrontGate = new Pose2d(24, -8, Math.toRadians(0));
    private final Pose2d redBackGate = new Pose2d(-24, -8, Math.toRadians(0));

    private int _pose;


    SampleMecanumDrive drive;


    @Override
    public void runOpMode() throws InterruptedException {
        drive.initArm();


        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(redBackStart);
        if (_pose == 0) {
            drive.setPoseEstimate(redBackStart);
            TrajectorySequence placePurpleBack = drive.trajectorySequenceBuilder(redBackStart)
                    .lineToSplineHeading(leftLine)
                    .build();
            drive.followTrajectorySequence(placePurpleBack);
            drive.setFrontGrip(false);
            sleep(1000);
            TrajectorySequence gotoback = drive.trajectorySequenceBuilder(leftLine)
                    .lineToSplineHeading(redBackTruss)
                    .lineToSplineHeading(redBackEnd)
                    .build();
            drive.followTrajectorySequence(gotoback);

        }

        else if (_pose == 1) {
            drive.setPoseEstimate(redBackStart);
            TrajectorySequence placePurpleBack = drive.trajectorySequenceBuilder(redBackStart)
                    .lineToSplineHeading(centerLine)
                    .build();
            drive.followTrajectorySequence(placePurpleBack);
            drive.setFrontGrip(false);
            sleep(1000);
            TrajectorySequence gotoback = drive.trajectorySequenceBuilder(centerLine)
                    .lineToSplineHeading(redBackGate)
                    .lineToSplineHeading(redFrontGate)
                    .lineToSplineHeading(redBackEnd)
                    .build();
            drive.followTrajectorySequence(gotoback);
        }

        else if (_pose == 2) {
            drive.setPoseEstimate(redBackStart);
            TrajectorySequence placePurpleBack = drive.trajectorySequenceBuilder(redBackStart)
                    .lineToSplineHeading(rightLine)
                    .build();
            drive.followTrajectorySequence(placePurpleBack);
            drive.setFrontGrip(false);
            sleep(1000);
            TrajectorySequence gotoback = drive.trajectorySequenceBuilder(rightLine)
                    .lineToSplineHeading(redBackTruss)
                    .lineToSplineHeading(redBackEnd)
                    .build();
            drive.followTrajectorySequence(gotoback);

        }

    }
}
