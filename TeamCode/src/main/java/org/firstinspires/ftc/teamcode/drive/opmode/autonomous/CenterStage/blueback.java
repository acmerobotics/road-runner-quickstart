package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.CenterStage;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
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




    private final Pose2d rightLine = new Pose2d(-48, 32, Math.toRadians(0));
    private final Pose2d  leftLine = new Pose2d(-24, 32, Math.toRadians(0));
    private final Pose2d centerLine = new Pose2d(-36, 24.5, Math.toRadians(-90));

    // truss
    private final Pose2d blueBackTruss = new Pose2d(-24, 36, Math.toRadians(0));
    private final Pose2d blueFrontGate = new Pose2d(24, 8, Math.toRadians(0));
    private final Pose2d blueBackGate = new Pose2d(-24, 8, Math.toRadians(0));

    private int _pose;

    SampleMecanumDrive drive;


    @Override
    public void runOpMode() throws InterruptedException {



        drive.initArm();


        drive = new SampleMecanumDrive(hardwareMap);
        // ask gram is second trajectory sequence needs to be the where the last trajectory sequence ended
        if (_pose == 0) {
            drive.setPoseEstimate(blueBackStart);
            TrajectorySequence placePurpleBack = drive.trajectorySequenceBuilder(blueBackStart)
                    .lineToSplineHeading(leftLine)
                    .build();
            drive.followTrajectorySequence(placePurpleBack);
            drive.setFrontGrip(false);
            sleep(1000);
            TrajectorySequence gotoback = drive.trajectorySequenceBuilder(leftLine)
                    .lineToSplineHeading(blueBackTruss)
                    .lineToSplineHeading(blueBackEnd)
                    .build();
            drive.followTrajectorySequence(gotoback);

        }

        else if (_pose == 1) {
            drive.setPoseEstimate(blueBackStart);
            TrajectorySequence placePurpleBack = drive.trajectorySequenceBuilder(blueBackStart)
                    .lineToSplineHeading(centerLine)
                    .build();
            drive.followTrajectorySequence(placePurpleBack);
            drive.setFrontGrip(false);
            sleep(1000);
            TrajectorySequence gotoback = drive.trajectorySequenceBuilder(centerLine)
                    .lineToSplineHeading(blueBackGate)
                    .lineToSplineHeading(blueFrontGate)
                    .lineToSplineHeading(blueBackEnd)
                    .build();
            drive.followTrajectorySequence(gotoback);
        }

        else if (_pose == 2) {
            drive.setPoseEstimate(blueBackStart);
            TrajectorySequence placePurpleBack = drive.trajectorySequenceBuilder(blueBackStart)
                    .lineToSplineHeading(rightLine)
                    .build();
            drive.followTrajectorySequence(placePurpleBack);
            drive.setFrontGrip(false);
            sleep(1000);
            TrajectorySequence gotoback = drive.trajectorySequenceBuilder(rightLine)
                    .lineToSplineHeading(blueBackTruss)
                    .lineToSplineHeading(blueBackEnd)
                    .build();
            drive.followTrajectorySequence(gotoback);

        }

    }
}
