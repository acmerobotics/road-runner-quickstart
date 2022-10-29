package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Timer;

@Autonomous(name = "RedCornerBlueSideAuto", group = "Auto")
public class RedCornerBlueSideAuto extends LinearOpMode{
    private Vision detector;
    private SampleMecanumDrive drivetrain;
    private Claw claw;
    private Lift lift;

    private void getSubsystems() {
        final Servo clawServo = hardwareMap.get(Servo.class, "clawServo");
        this.claw = new Claw(clawServo);

        final DcMotor liftMotorLeft = hardwareMap.get(DcMotor.class, "liftMotorLeft");
        final DcMotor liftMotorRight = hardwareMap.get(DcMotor.class, "liftMotorRight");
        this.lift = new Lift(liftMotorLeft, liftMotorRight);

        this.detector = new Vision(telemetry);
        this.drivetrain = new SampleMecanumDrive(hardwareMap);
    }

    public void scoreConeInClaw(
            final int liftStage, final Trajectory score
    ) {
        lift.liftToStage(liftStage);

        drivetrain.followTrajectory(score);
        drivetrain.waitForIdle();

        claw.clawOpen();
        final long initialT = System.currentTimeMillis();
        while (System.currentTimeMillis() < (initialT + 2000)) {
            if (isStopRequested())
                break;
        }

        lift.retract();
        claw.clawClose();
    }

    public void scoreConeInClaw(
            final int liftStage, final Trajectory score, final Trajectory endPosition
    ) {
        scoreConeInClaw(liftStage, score);
        drivetrain.followTrajectory(endPosition);
        drivetrain.waitForIdle();
    }

    private void grabCone(final Trajectory depot) {
        claw.clawOpen();

        drivetrain.followTrajectory(depot);
        drivetrain.waitForIdle();
        claw.clawClose();
    }

    private void grabAndScoreCone(
            final int liftStage,
            final Trajectory depot,
            final Trajectory score,
            final Trajectory endPosition
    ) {
        grabCone(depot);
        scoreConeInClaw(liftStage, score, endPosition);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        getSubsystems();

        //1 for blue corner blue side
        //2 for blue corner red side
        //3 for red corner red side
        //4 for red corner blue side

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

        final Trajectory depotPath =
                drivetrain.trajectoryBuilder(new Pose2d(35,63.5,-90))
                    .splineToLinearHeading(new Pose2d(35,18,90),-90)
                    .splineToLinearHeading(new Pose2d(55,12,90),-180)
                    .build();
        final Trajectory scorePath = null; //TODO: add scoring path to junction here

        final Trajectory parkToCenter = drivetrain.trajectoryBuilder(depotPath.end())
                .splineToLinearHeading(new Pose2d(36,37),-180)
                .build();
        final Trajectory parkToRight = drivetrain.trajectoryBuilder(depotPath.end())
                .splineToLinearHeading(new Pose2d(36,37),-90)
                .splineToLinearHeading(new Pose2d(60,37),-180)
                .build();
        final Trajectory parkToLeft = drivetrain.trajectoryBuilder(depotPath.end())
                .splineToLinearHeading(new Pose2d(36,37),-90)
                .splineToLinearHeading(new Pose2d(10,37),-180)
                .build();

        //Grab and then score a cone to a junction at lift stage 3, using starting depot path,
        //score path, and then park to the center parking area
        //TODO: actually do this correctly using the SleeveDirection
        //TODO: change liftStage to the correct liftStage
        //TODO: change revolutions in Lift to match correct revolutions to each stage
        grabAndScoreCone(3, depotPath, scorePath, parkToCenter);

        /*
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
         */
    }
}