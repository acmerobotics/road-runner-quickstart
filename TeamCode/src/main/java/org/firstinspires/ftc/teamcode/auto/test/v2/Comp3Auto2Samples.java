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

import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Armv2;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Liftv2;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Wristv2;


@Config
@Autonomous(name = "Comp3Auto2Samples", group = "RoadRunner 1.0")

public class Comp3Auto2Samples extends LinearOpMode {

    public static double RED_BASKET_POS_X = -52;
    ;
    public static double RED_BASKET_POS_Y = -52;
    public static double RED_BASKET_ANGLE = Math.toRadians(45);

    public static double RED_SAMPLE1_POS_X = -42.5;
    public static double RED_SAMPLE1_POS_Y = -36;
    public static double RED_SAMPLE1_ANGLE = Math.toRadians(90);

    public static double RED_SAMPLE2_POS_X = RED_SAMPLE1_POS_X - 10;
    public static double RED_SAMPLE3_POS_X = RED_SAMPLE2_POS_X - 3.5;
    public static double RED_SAMPLE2_POS_Y = RED_SAMPLE1_POS_Y;
    public static double RED_SAMPLE3_POS_Y = RED_SAMPLE2_POS_Y - 1;
    public static double RED_SAMPLE2_ANGLE = RED_SAMPLE1_ANGLE;
    public static double RED_SAMPLE3_ANGLE = Math.toRadians(120);

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

        arm.reset();
        lift.reset();


        Action scoreHighAction3 = new ParallelAction(
                arm.armScoreAction(),
                lift.liftUpAction(),
                wrist.wristVerticalAction()
        );

        Action comeDownAction = new SequentialAction(
                arm.armComeDownAction(),
                new ParallelAction(
                        wrist.wristFoldInAction(),
                        lift.liftDownAction()
                )
        );

        Action comeDownAction2 = new SequentialAction(
                arm.armComeDownAction(),
                wrist.wristVerticalAction(),
                lift.liftDownAction(),
                new ParallelAction(
                        arm.armPickupGroundSampleAction(),
                        wrist.wristFoldInAction()
                )
        );

        Action comeDownAction3 = new SequentialAction(
                arm.armComeDownAction(),
                wrist.wristVerticalAction(),
                lift.liftDownAction(),
                new ParallelAction(
                        arm.armPickupGroundSampleAction(),
                        wrist.wristFoldInAction()
                )
        );

        Action comeDownAction4 = new SequentialAction(
                arm.armComeDownAction(),
                wrist.wristVerticalAction(),
                lift.liftDownAction(),
                new ParallelAction(
                        arm.armPickupGroundSampleAction(),
                        wrist.wristFoldInAction()
                )
        );



        TrajectoryActionBuilder driveToBasket = drive.actionBuilder(RED_SCORE_START_POSE)
                .strafeToLinearHeading(new Vector2d(-36, -56), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(RED_BASKET_POS_X, RED_BASKET_POS_Y), RED_BASKET_ANGLE);
        TrajectoryActionBuilder driveToSample1 = drive.actionBuilder(RED_NEAR_BASKET_POSE)
                .strafeToLinearHeading(new Vector2d(RED_SAMPLE1_POS_X, RED_SAMPLE1_POS_Y), RED_SAMPLE1_ANGLE);
        TrajectoryActionBuilder driveToSample2 = drive.actionBuilder(RED_NEAR_BASKET_POSE)
                .strafeToLinearHeading(new Vector2d(RED_SAMPLE2_POS_X, RED_SAMPLE2_POS_Y), RED_SAMPLE2_ANGLE);
        TrajectoryActionBuilder driveToSample3 = drive.actionBuilder(RED_NEAR_BASKET_POSE)
                .strafeToLinearHeading(new Vector2d(RED_SAMPLE3_POS_X, RED_SAMPLE3_POS_Y), RED_SAMPLE3_ANGLE);
        TrajectoryActionBuilder driveBasketToPark =  drive.actionBuilder(RED_NEAR_BASKET_POSE)
                .strafeToLinearHeading(new Vector2d(-33,-9),Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-22,-9),Math.toRadians(180));

        TrajectoryActionBuilder driveSample1ToBasket = drive.actionBuilder(RED_SAMPLE1_POSE)
                .strafeToLinearHeading(new Vector2d(RED_BASKET_POS_X, RED_BASKET_POS_Y), RED_BASKET_ANGLE);
        TrajectoryActionBuilder driveSample2ToBasket = drive.actionBuilder(RED_SAMPLE2_POSE)
                .strafeToLinearHeading(new Vector2d(RED_BASKET_POS_X, RED_BASKET_POS_Y), RED_BASKET_ANGLE);
        TrajectoryActionBuilder driveSample3ToBasket = drive.actionBuilder(RED_SAMPLE3_POSE)
                .strafeToLinearHeading(new Vector2d(RED_BASKET_POS_X, RED_BASKET_POS_Y), RED_BASKET_ANGLE);

        // ==== test only come back at the end
        TrajectoryActionBuilder goBackToStart = driveToBasket.endTrajectory().fresh()
                .strafeToLinearHeading(RED_SCORE_START_POSE.position, 0);
        Action cGoBackToStartAction = new ParallelAction(
                goBackToStart.build(),
                claw.clawOpenAction(),
                arm.armResetAction(),
                wrist.wristFoldInAction());
        // ==== end test only

        // ===== Go to Basket and Score
        Action cStartToBasketAction = new SequentialAction(
                new ParallelAction(
                        driveToBasket.build(),
                        arm.armScoreAction(),
                        wrist.wristVerticalAction()),
                lift.liftUpAction(),
                //wrist.wristFoldOutAction(),
                new SleepAction(0.3),
                claw.clawOpenAction(),
                new SleepAction(0.2)
        );
        Action cSample1ToBasketAction = new SequentialAction(
                new ParallelAction(
                        driveSample1ToBasket.build(),
                        arm.armScoreAction(),
                        wrist.wristVerticalAction()),
                lift.liftUpAction(),
                //wrist.wristFoldOutAction(),
                new SleepAction(0.3),
                claw.clawOpenAction(),
                new SleepAction(0.2)
        );

        Action cSample2ToBasketAction = new SequentialAction(
                new ParallelAction(
                        driveSample2ToBasket.build(),
                        arm.armScoreAction(),
                        wrist.wristVerticalAction()),
                lift.liftUpAction(),
                //wrist.wristFoldOutAction(),
                new SleepAction(0.3),
                claw.clawOpenAction(),
                new SleepAction(0.2)
        );

        Action cSample3ToBasketAction = new SequentialAction(
                new ParallelAction(
                        driveSample3ToBasket.build(),
                        arm.armScoreAction(),
                        wrist.wristVerticalAction()),
                lift.liftUpAction(),
                //wrist.wristFoldOutAction(),
                new SleepAction(0.3),
                claw.clawOpenAction(),
                new SleepAction(0.2)
        );


        // ====== Basket --> sample, park etc.
        Action cBasketToSample1Action = new ParallelAction(
                driveToSample1.build(),
                arm.armPickupGroundSampleAction(),
                wrist.wristFoldInAction(),
                lift.liftDownAction()
        );

        Action cBasketToSample2Action = new ParallelAction(
                driveToSample2.build(),
                arm.armPickupGroundSampleAction(),
                wrist.wristFoldInAction(),
                lift.liftDownAction()
        );

        Action cBasketToSample3Action = new ParallelAction(
                driveToSample3.build(),
                arm.armPickupGroundSampleAction(),
                wrist.wristFoldInAction(),
                lift.liftDownAction()
        );


        Action comeDownActionForPark = new SequentialAction(
                arm.armComeDownAction(),
                wrist.wristVerticalAction(),
                lift.liftDownAction(),
                arm.armResetAction()
        );

        Action cBasketToParkAction = new ParallelAction(
                comeDownActionForPark,
                driveBasketToPark.build()
        );

        while (!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        // start to basket and score
                        cStartToBasketAction
                        // basket to sample 1
                        ,comeDownAction2
                        ,cBasketToSample1Action
                        ,new SleepAction(.4)
                        ,claw.clawCloseAction()
                        ,new SleepAction(1)
                        // sample 1 to basket
                        ,cSample1ToBasketAction
                        ,comeDownAction3
                        // basket to sample 2
                        ,cBasketToSample2Action
                        ,new SleepAction(.2)
                        ,claw.clawCloseAction()
                        ,new SleepAction(0.5)
                        // sample 2 to basket
                        ,cSample2ToBasketAction
                        // park
                        ,cBasketToParkAction
                )
        );
    } // runOpMode


}


