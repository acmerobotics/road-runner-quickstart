package org.firstinspires.ftc.teamcode.auto.test.v2;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Armv2;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Liftv2;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Wristv2;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Robotv2;


@Config
@Autonomous(name = "Comp4Auto", group = "RoadRunner 1.0")

public class Comp4Auto extends LinearOpMode {

    public static double PRE_DROP_SLEEP = 1;
    public static double POST_DROP_SLEEP = 1;
    public static double RED_BASKET_POS_X = -46.5;
    public static double RED_BASKET_POS_Y = -46.5;
    public static double RED_BASKET_ANGLE = Math.toRadians(45);

    public static double RED_SAMPLE1_POS_X = -43.2;
    public static double RED_SAMPLE1_POS_Y = -46.7;
    public static double RED_SAMPLE1_ANGLE = Math.toRadians(90);

    public static double RED_SAMPLE2_POS_X = RED_SAMPLE1_POS_X - 10;
    public static double RED_SAMPLE2_POS_Y = RED_SAMPLE1_POS_Y;
    public static double RED_SAMPLE2_ANGLE = RED_SAMPLE1_ANGLE;

    public static double RED_SAMPLE3_POS_X = -41;
    public static double RED_SAMPLE3_POS_Y = -41;
    public static double RED_SAMPLE3_ANGLE = Math.toRadians(135);

    public static double RED_SAMPLE3_POS_X_2 = -46;
    public static double RED_SAMPLE3_POS_Y_2 = -41;


    // Start position red near
    Pose2d RED_SCORE_START_POSE = new Pose2d(-36, -59, Math.toRadians(0));

    Pose2d RED_NEAR_BASKET_POSE = new Pose2d(RED_BASKET_POS_X, RED_BASKET_POS_Y, RED_BASKET_ANGLE);

    Pose2d RED_SAMPLE1_POSE = new Pose2d(RED_SAMPLE1_POS_X, RED_SAMPLE1_POS_Y, RED_SAMPLE1_ANGLE);

    Pose2d RED_SAMPLE2_POSE = new Pose2d(RED_SAMPLE2_POS_X, RED_SAMPLE2_POS_Y, RED_SAMPLE2_ANGLE);

    Pose2d RED_SAMPLE3_POSE = new Pose2d(RED_SAMPLE3_POS_X, RED_SAMPLE3_POS_Y, RED_SAMPLE3_ANGLE);


    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, RED_SCORE_START_POSE);
        Armv2 arm = new Armv2(hardwareMap);
        Liftv2 lift = new Liftv2(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Wristv2 wrist = new Wristv2(hardwareMap);
        Robotv2 robot = new Robotv2(hardwareMap, RED_SCORE_START_POSE);

        arm.reset();
        lift.reset();

        TrajectoryActionBuilder driveToBasket = drive.actionBuilder(RED_SCORE_START_POSE)
                .strafeToLinearHeading(new Vector2d(-36, -56), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(RED_BASKET_POS_X, RED_BASKET_POS_Y), RED_BASKET_ANGLE);
        TrajectoryActionBuilder driveToSample1 = drive.actionBuilder(RED_NEAR_BASKET_POSE)
                .strafeToLinearHeading(new Vector2d(RED_SAMPLE1_POS_X, RED_SAMPLE1_POS_Y), RED_SAMPLE1_ANGLE);
        TrajectoryActionBuilder driveToSample2 = drive.actionBuilder(RED_NEAR_BASKET_POSE)
                .strafeToLinearHeading(new Vector2d(RED_SAMPLE2_POS_X, RED_SAMPLE2_POS_Y), RED_SAMPLE2_ANGLE);
        TrajectoryActionBuilder driveToSample3_1 = drive.actionBuilder(RED_NEAR_BASKET_POSE)
                .strafeToLinearHeading(new Vector2d(RED_SAMPLE3_POS_X, RED_SAMPLE3_POS_Y), RED_SAMPLE3_ANGLE);
        TrajectoryActionBuilder driveToSample3_2 = driveToSample3_1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(RED_SAMPLE3_POS_X_2, RED_SAMPLE3_POS_Y_2), RED_SAMPLE3_ANGLE);

        TrajectoryActionBuilder driveSample1ToBasket = drive.actionBuilder(RED_SAMPLE1_POSE)
                .strafeToLinearHeading(new Vector2d(RED_BASKET_POS_X - 1, RED_BASKET_POS_Y - 1), RED_BASKET_ANGLE);
        TrajectoryActionBuilder driveSample2ToBasket = drive.actionBuilder(RED_SAMPLE2_POSE)
                .strafeToLinearHeading(new Vector2d(RED_BASKET_POS_X - 2, RED_BASKET_POS_Y - 2), RED_BASKET_ANGLE);
        TrajectoryActionBuilder driveSample3ToBasket = drive.actionBuilder(RED_SAMPLE3_POSE)
                .strafeToLinearHeading(new Vector2d(RED_BASKET_POS_X - 3, RED_BASKET_POS_Y - 3), RED_BASKET_ANGLE);


        // ===== Go to Basket and Score
        Action cStartToBasketAction = new SequentialAction(
                new ParallelAction(
                        wrist.wristFoldInAction(),
                        driveToBasket.build(),
                        arm.armScoreAction(),
                        new SequentialAction(
                                new SleepAction(1),
                                lift.liftUpAction()
                        )
                ),
                wrist.wristFoldOutAction(),
                new SleepAction(PRE_DROP_SLEEP),
                claw.clawOpenAction(),
                new SleepAction(POST_DROP_SLEEP),
                wrist.wristFoldInAction(),
                new SleepAction(POST_DROP_SLEEP)
        );
        Action cSample1ToBasketAction = new SequentialAction(
                new ParallelAction(
                        driveSample1ToBasket.build(),
                        robot.scoreSampleActionAuto()
                        )
                );

        Action cSample2ToBasketAction = new SequentialAction(
                new ParallelAction(
                        driveSample2ToBasket.build(),
                        robot.scoreSampleActionAuto()
                        )
                );

        Action cSample3ToBasketAction = new SequentialAction(
                new ParallelAction(
                        driveSample3ToBasket.build(),
                        robot.scoreSampleActionAuto()
                        )
                );


        // ====== Basket --> sample, park etc.
        Action cBasketToSample1Action = new SequentialAction(
                new ParallelAction(
                        driveToSample1.build(),
                        robot.comeDownActionAuto()),
                robot.pickUpActionAuto());

        Action cBasketToSample2Action = new SequentialAction(
                new ParallelAction(
                        driveToSample2.build(),
                        robot.comeDownActionAuto()),
                robot.pickUpActionAuto());

        Action cBasketToSample3Action = new SequentialAction(
                new ParallelAction(
                        driveToSample3_1.build(),
                        robot.comeDownSample3ActionAuto()),
                driveToSample3_2.build(),
                robot.arm.armPickupGroundSampleLiftOutAction(),
                robot.pickUpActionAuto(),
                new SleepAction(0.2));

        while (!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        // start to basket and score
                        cStartToBasketAction
                        // basket to sample 1
                        ,cBasketToSample1Action
                        // sample 1 to basket
                        ,cSample1ToBasketAction
                        // basket to sample 2
                        ,cBasketToSample2Action
                        // sample 2 to basket
                        ,cSample2ToBasketAction
                        // basket to sample 3
                        ,cBasketToSample3Action
                        // sample 3 to basket
                        ,cSample3ToBasketAction
                        // Reset arm and slide (lift)
                        ,robot.resetAction()
                )
        );
    } // runOpMode


}


