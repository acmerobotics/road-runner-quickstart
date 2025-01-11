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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Armv2;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Liftv2;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Wristv2;
import org.firstinspires.ftc.teamcode.mechanisms.v1.Wrist;


@Config
@Autonomous(name = "Arm+Drive testv2", group = "RoadRunner 1.0")

public class ArmDriveTestv2 extends LinearOpMode {


    // Start position red near
    Pose2d RED_SCORE_START_POSE = new Pose2d(-36, -59, Math.toRadians(0));

    Pose2d RED_NEAR_BASKET_POSE = new Pose2d(-50, -50, Math.toRadians(45));

    Pose2d RED_SAMPLE1_POSE = new Pose2d(-44.5, -36, Math.toRadians(90));

    Pose2d RED_SAMPLE2_POSE = new Pose2d(-54.5, -36, Math.toRadians(90));

    Pose2d RED_SAMPLE3_POSE = new Pose2d(-60, -24, Math.toRadians(180));




    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, RED_SCORE_START_POSE);
        Armv2 arm = new Armv2(hardwareMap);
        Liftv2 lift = new Liftv2(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Wristv2 wrist = new Wristv2(hardwareMap);

        arm.reset();

//        Action scoreHighAction2 = new ParallelAction(
//                arm.armVerticalAction(),
//                wrist.wristFoldOutAction()
//        );

        Action scoreHighAction3 = new ParallelAction(
                arm.armScoreAction(),
                lift.liftUpAction(),
                wrist.wristVerticalAction()
        );

        Action comedownAction = new SequentialAction(
          arm.armComeDownAction(),
          new ParallelAction(
                  wrist.wristFoldInAction(),
                  lift.liftDownAction()
          )
        );
        Action pickupsample = new SequentialAction(
                claw.clawOpenAction()
        );




        TrajectoryActionBuilder drivetobasket = drive.actionBuilder(RED_SCORE_START_POSE)
                .strafeToLinearHeading(new Vector2d(-36, -56), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(45));
        TrajectoryActionBuilder drivetosample1 = drive.actionBuilder(RED_NEAR_BASKET_POSE)
                .strafeToLinearHeading(new Vector2d(-44.5, -36), Math.toRadians(90));
        TrajectoryActionBuilder drivetosample2 = drive.actionBuilder(RED_NEAR_BASKET_POSE)
                .strafeToLinearHeading(new Vector2d(-54.5, -36), Math.toRadians(90));
        TrajectoryActionBuilder drivetosample3 = drive.actionBuilder(RED_NEAR_BASKET_POSE)
                .strafeToLinearHeading(new Vector2d(-60, -24), Math.toRadians(180));

        TrajectoryActionBuilder sample1tobasket = drive.actionBuilder(RED_SAMPLE1_POSE)
                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(45));
        TrajectoryActionBuilder sample2tobasket = drive.actionBuilder(RED_SAMPLE2_POSE)
                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(45));
        TrajectoryActionBuilder sample3tobasket = drive.actionBuilder(RED_SAMPLE3_POSE)
                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(45));

        // test only come back at the end
        TrajectoryActionBuilder goBackToStart = drivetobasket.endTrajectory().fresh()
                .strafeToLinearHeading(RED_SCORE_START_POSE.position, 0);
        Action cGoBackToStartAction = new ParallelAction(
                goBackToStart.build(),
                claw.clawOpenAction(),
                arm.armResetAction(),
                wrist.wristFoldInAction());
        // end test only


//        Action cStartToBasketScoreAction = new ParallelAction(drivetobasket.build(), scoreHighAction2);


        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

//        Actions.runBlocking
//                ( new SequentialAction(
//                        cStartToBasketScoreAction,
//                        lift.liftUpAction(),
//                        arm.armScoreAction(),
//                        claw.clawOpenAction(),
//                        comedownAction
//
//                ));

        Action cStartToBasketScoreAction3 = new ParallelAction(drivetobasket.build(), scoreHighAction3);

        Action cBasketToSample1Action = new ParallelAction(
                drivetosample1.build(),
                arm.armPickupGroundSampleAction(),
                wrist.wristFoldInAction(),
                lift.liftDownAction()
        );

        Action cStartToBasketAction = new SequentialAction(
                new ParallelAction(
                        drivetobasket.build(),
                        arm.armScoreAction(),
                        wrist.wristVerticalAction()),
                lift.liftUpAction(),
                wrist.wristFoldOutAction(),
                new SleepAction(0.5),
                claw.clawOpenAction(),
                new SleepAction(0.2)
        );

        Action cSample1ToBasketAction = new SequentialAction(
                new ParallelAction(
                        sample1tobasket.build(),
                        arm.armScoreAction()),
                new ParallelAction(
                        lift.liftUpAction(),
                        wrist.wristVerticalAction())
        );



        Actions.runBlocking(
                new SequentialAction(
                        // start to basket and score
                        cStartToBasketAction,
//                        cStartToBasketScoreAction3,
//                        wrist.wristFoldOutAction(),
//                        new SleepAction(0.5),
//                        claw.clawOpenAction(),
//                        new SleepAction(.2),
                        comedownAction,
                        cBasketToSample1Action,
                        claw.clawCloseAction(),
                        new SleepAction(.5),
                        cSample1ToBasketAction,
                        wrist.wristFoldOutAction(),
                        new SleepAction(1),
                        claw.clawOpenAction(),
                        new SleepAction(.2),
                        // Come down action
                        new SequentialAction(
                                arm.armComeDownAction(),
                                new ParallelAction(
                                        wrist.wristFoldInAction(),
                                        lift.liftDownAction()
                                ))
                        ));
                        // basket to sample 2
                        // ...
                        // sample 2 to basket
                        // ...
                        // basket to sample 3
                        // ...
                        // sample 3 to basket
                        // ...
                        // park
                        // ...



        ;
    } // runOpMode



}


