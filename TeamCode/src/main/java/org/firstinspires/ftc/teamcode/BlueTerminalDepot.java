package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Timer;

public class BlueTerminalDepot extends LinearOpMode{
    private Vision detector;
    private SampleMecanumDrive drivetrain;
    private Claw claw;

    private void grabAndScoreCone(final Trajectory depot, final Trajectory score) {
        claw.clawOpen();
        drivetrain.followTrajectory(depot);
        drivetrain.waitForIdle();
        claw.clawClose();
        drivetrain.followTrajectory(score);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        final Servo clawServo = hardwareMap.get(Servo.class, "clawServo");
        this.claw = new Claw(clawServo);

        this.detector = new Vision(telemetry);
        this.drivetrain = new SampleMecanumDrive(hardwareMap);

        final Trajectory signalLeftPath;
        final Trajectory signalRightPath;
        final Trajectory signalCenterPath;
        final Trajectory depotPath;

        //1 for blue terminal with depot
        //2 for blue without depot
        //3 for red with depot
        //4 for red without depot

        waitForStart();
        final Vision.SleeveDirection sleeveDirection;
        final long sysTimeMilli = System.currentTimeMillis();
        while (System.currentTimeMillis() < (sysTimeMilli + 10000)) {
            final Vision.SleeveDirection direction = detector.getSleeveDirection();
            if (direction != null) { //TODO: replace with SleeveDirection.NONE
                sleeveDirection = direction;
                break;
            }
        }




        if (fieldPosition == 1){
                signalCenterPath = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
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
            depotPath = drivetrain.trajectoryBuilder(new Pose2d(35,63.5,-90))
                    .splineToLinearHeading(new Pose2d(35,18,90),-90)
                    .splineToLinearHeading(new Pose2d(55,12,90),-180)
                    .build();
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
        drivetrain.followTrajectory(depotPath);
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