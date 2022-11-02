package org.firstinspires.ftc.teamcode.paths;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.claw.Claw;
import org.firstinspires.ftc.teamcode.lift.Lift;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "BlueCornerBlueSideAuto_DO_NOT_RUN", group = "Auto")
public class BlueCornerBlueSideAuto extends LinearOpMode {
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

        final Trajectory depotPath =
                drivetrain.trajectoryBuilder(new Pose2d(35, 63.5, -90))
                        .splineToLinearHeading(new Pose2d(35, 18, 90), -90)
                        .splineToLinearHeading(new Pose2d(55, 12, 90), -180)
                        .build();
        final Trajectory scorePath = null; //TODO: add scoring path to junction here

        final Trajectory parkToCenter = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                .splineToLinearHeading(new Pose2d(-36, 37), -180)
                .build();
        final Trajectory parkToRight = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                .splineToLinearHeading(new Pose2d(-36, 37), -90)
                .splineToLinearHeading(new Pose2d(-60, 37), -180)
                .build();
        final Trajectory parkToLeft = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                .splineToLinearHeading(new Pose2d(-36, 37), -90)
                .splineToLinearHeading(new Pose2d(-10, 37), -180)
                .build();
        final Trajectory trajectory = drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                .splineToLinearHeading(new Pose2d(30, 37, -90), -90)
                .splineToLinearHeading(new Pose2d(-35, 37, -90), -180)
                .build();

        //Grab and then score a cone to a junction at lift stage 3, using starting depot path,
        //score path, and then park to the center parking area
        //TODO: actually do this correctly using the SleeveDirection
        //TODO: change liftStage to the correct liftStage
        //TODO: change revolutions in Lift to match correct revolutions to each stage
        grabAndScoreCone(3,
                depotPath,
                scorePath,
                parkToCenter);

    }

}