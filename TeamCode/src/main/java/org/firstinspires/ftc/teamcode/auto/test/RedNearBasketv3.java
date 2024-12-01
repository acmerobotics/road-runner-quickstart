package org.firstinspires.ftc.teamcode.auto.test;
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
import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.Wrist;


@Config
@Autonomous(name = "\uD83D\uDD34 - RedNearBasketV3", group = "RoadRunner 1.0")
public class RedNearBasketv3 extends LinearOpMode {


    // Start position red near
    Pose2d RED_SCORE_START_POSE = new Pose2d(-38, -60, Math.toRadians(180));

    public static double RED_BASKET_X = -48;
    public static double RED_BASKET_Y = -48;
    public static double RED_BASKET_HEADING = 180+45;

    public static double RED_SAMPLE1_X = -28;
    public static double RED_SAMPLE2_X = -36;
    public static double RED_SAMPLE3_X = -42; // -46 would hit the boundary

    public static double RED_SAMPLE1_Y = -30;
    public static double RED_SAMPLE2_Y = -22;
    public static double RED_SAMPLE3_Y = -22;

    public static double RED_SAMPLE1_HEADING = 161;
    public static double RED_SAMPLE2_HEADING = 180;
    public static double RED_SAMPLE3_HEADING = 180;


    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, RED_SCORE_START_POSE);
        Intake intake = new Intake(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);

        //==== Start of Non-trajectory related actions ====
        // TODO: Move to a shared file
        Action scoreHighAction = new SequentialAction(
                new ParallelAction(
                wrist.wristFoldOutAction(),
                arm.armScoreAction()),
                lift.liftUpAction()
        );
//        Action scoreHighAction2 = new ParallelAction(
//                arm.armScoreAction(),
//                lift.liftUpAction()
//        );
        Action foldBackAction = new ParallelAction(
                arm.armfoldbackaction(),
                lift.liftDownAction()
        );
//        Action armUpAction = new ParallelAction(
//                arm.armRobotTravelAction(),
//                lift.liftDownAction()
//        );
        Action collectAction = new SequentialAction(
                arm.armGroundCollectAction(),
                intake.intakeAction()
        );
//
//        public Action MechanismWhenGoingToSample(){
//            Action robotTravelActionTest = new SequentialAction(
//                    arm.armVerticalAction()
//                    ,lift.liftDownAction()
//                    ,arm.armRobotTravelAction()
//            );
//        }

        // ==== End of Non-trajectory related actions ====

        // ==== Start of Trajectory actions ====
        TrajectoryActionBuilder startToBasketTab = drive.actionBuilder(RED_SCORE_START_POSE)
                .strafeToLinearHeading(new Vector2d(-38, -56), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(RED_BASKET_X, RED_BASKET_Y), Math.toRadians(RED_BASKET_HEADING));

        TrajectoryActionBuilder driveBasketToSample1Tab = startToBasketTab.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(-30, RED_SAMPLE1_Y), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(RED_SAMPLE1_X, RED_SAMPLE1_Y), Math.toRadians(RED_SAMPLE1_HEADING));

        TrajectoryActionBuilder driveSample1ToBasketTab = driveBasketToSample1Tab.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(RED_BASKET_X, RED_BASKET_Y), Math.toRadians(RED_BASKET_HEADING));

        TrajectoryActionBuilder driveBasketToSample2Tab = driveSample1ToBasketTab.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(RED_SAMPLE2_X, RED_SAMPLE2_Y), Math.toRadians(180));

        TrajectoryActionBuilder driveSample2ToBasketTab = driveBasketToSample2Tab.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(RED_BASKET_X, RED_BASKET_Y), Math.toRadians(RED_BASKET_HEADING));

        TrajectoryActionBuilder driveBasketToSample3Tab = driveSample2ToBasketTab.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(RED_SAMPLE3_X, RED_SAMPLE3_Y), Math.toRadians(180));

        TrajectoryActionBuilder driveSample3ToBasketTab = driveBasketToSample3Tab.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(RED_BASKET_X, RED_BASKET_Y), Math.toRadians(RED_BASKET_HEADING));

        // ==== End of Trajectory actions ====


        //  ==== Start of composite actions: combine trajectory and non-trajectory actions
        Action cStartToBasketScoreAction = new ParallelAction(startToBasketTab.build(), scoreHighAction);

//        Action robotTravelActionTest = new SequentialAction(
//                arm.armVerticalAction()
//                ,lift.liftDownAction()
//                ,arm.armRobotTravelAction()
//        );
        Action cBasketToSample1Action = new ParallelAction(
                new SequentialAction(
                        arm.armVerticalAction()
                        ,lift.liftDownAction()
                        ,arm.armRobotTravelAction()
                ),
                driveBasketToSample1Tab.build()
        );

        Action cBasketToSample2Action = new ParallelAction(
                new SequentialAction(
                        arm.armVerticalAction()
                        ,lift.liftDownAction()
                        ,arm.armRobotTravelAction()
                ),
                driveBasketToSample2Tab.build()
        );
        Action cBasketToSample3Action = new ParallelAction(
//                arm.armRobotTravelAction(),
//                lift.liftDownAction(),
                new SequentialAction(
                        arm.armVerticalAction()
                        ,lift.liftDownAction()
                        ,arm.armRobotTravelAction()
                ),
                driveBasketToSample3Tab.build()
        );

        Action cSample1ToBasketAction = new ParallelAction(
                driveSample1ToBasketTab.build(),
                arm.armScoreAction(),
                lift.liftUpAction()
        );
        Action cSample2ToBasketAction = new ParallelAction(
                driveSample2ToBasketTab.build(),
                arm.armScoreAction(),
                lift.liftUpAction()
        );
        Action cSample3ToBasketAction = new ParallelAction(
                driveSample3ToBasketTab.build(),
                arm.armScoreAction(),
                lift.liftUpAction()
        );
        // ==== End of composite actions ====


        while(!isStopRequested() && !opModeIsActive()) {
            // Wait for the start signal
        }

        waitForStart();

        if (isStopRequested());

        Actions.runBlocking(
                new SequentialAction(
                        cStartToBasketScoreAction,
                        new SleepAction(0.1),
                        intake.depositAction(),
                        //
                        cBasketToSample1Action,
                        collectAction,
                        arm.armVerticalAction(),
                        cSample1ToBasketAction,
                        new SleepAction(0.1),
                        intake.depositAction(),
                        //
                        cBasketToSample2Action,
                        arm.armGroundCollectAction(),
                        new SleepAction(0.5),
                        arm.armVerticalAction(),
                        cSample2ToBasketAction,
                        new SleepAction(0.1),
                        intake.depositAction(),
                        //
//                        cBasketToSample3Action,
//                        arm.armGroundCollectAction(),
//                        new SleepAction(0.5),
//                        arm.armVerticalAction(),
//                        cSample3ToBasketAction,
//                        new SleepAction(0.1),
//                        intake.depositAction(),
                        //
                        foldBackAction
                )
        );

    } // runOpMode



}

