package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class signalParking extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        final Vision detector = new Vision(telemetry);
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
        final int fieldPosition = 1;
        Trajectory lineToLeft;
        Trajectory lineToRight;
        Trajectory lineToCenter;

        //1 for blue terminal with depot
        //2 for blue without depot
        //3 for red with depot
        //4 for red without depot

        waitForStart();
        if (fieldPosition == 1){
                lineToCenter = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                .splineToLinearHeading(new Pose2d(-36,37),-180)
                .build();
                lineToRight = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                        .splineToLinearHeading(new Pose2d(-36,37),-90)
                        .splineToLinearHeading(new Pose2d(-60,37),-180)
                        .build();
                lineToLeft = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                        .splineToLinearHeading(new Pose2d(-36,37),-90)
                        .splineToLinearHeading(new Pose2d(-10,37),-180)
                        .build();
                final Trajectory trajectory = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                        .splineToLinearHeading(new Pose2d(30, 37, -90), -90)
                        .splineToLinearHeading(new Pose2d(-35, 37, -90), -180)
                        .build();


        }
        else if (fieldPosition == 2){
            lineToCenter = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(36,-37),-180)
                    .build();
            lineToRight = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(36,-37),-90)
                    .splineToLinearHeading(new Pose2d(60,-37),-180)
                    .build();
            lineToLeft = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(36,-37),-90)
                    .splineToLinearHeading(new Pose2d(10,-37),-180)
                    .build();
                    //+-
        }
        else if (fieldPosition == 3){
            lineToCenter = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(-36,-37),-180)
                    .build();
            lineToRight = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(-36,-37),-90)
                    .splineToLinearHeading(new Pose2d(-60,-37),-180)
                    .build();
            lineToLeft = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(-36,-37),-90)
                    .splineToLinearHeading(new Pose2d(-10,-37),-180)
                    .build();
                   //--
        }
        else if (fieldPosition == 4){
            lineToCenter = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(36,37),-180)
                    .build();
            lineToRight = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(36,37),-90)
                    .splineToLinearHeading(new Pose2d(60,37),-180)
                    .build();
            lineToLeft = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(36,37),-90)
                    .splineToLinearHeading(new Pose2d(10,37),-180)
                    .build();
                    //++
        }
        switch (detector.getSleeveDirection()) {
            case LEFT:
                drivetrain.followTrajectory(lineToLeft);

            case RIGHT:
                drivetrain.followTrajectory(lineToRight);

            case CENTER:
                drivetrain.followTrajectory(lineToCenter);

        }
    }
}