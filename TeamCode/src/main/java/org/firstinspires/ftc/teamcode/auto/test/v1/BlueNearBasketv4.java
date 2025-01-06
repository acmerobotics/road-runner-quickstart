package org.firstinspires.ftc.teamcode.auto.test.v1;



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
import org.firstinspires.ftc.teamcode.mechanisms.v1.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.v1.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.v1.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.v1.Wrist;


@Config
@Autonomous(name = "\uD83D\uDD35 - RedNearBasketV4", group = "RoadRunner 1.0")
public class BlueNearBasketv4 extends LinearOpMode {


    // Start position red near
    Pose2d BLUE_SCORE_START_POSE = new Pose2d(-38, -60, Math.toRadians(180));

    public static double BLUE_BASKET_X = 47;
    public static double BLUE_BASKET_Y = 47;
    public static double BLUE_BASKET_HEADING = 45;

    public static double BLUE_SAMPLE1_X = 26;
    public static double BLUE_SAMPLE2_X = 35;
    public static double BLUE_SAMPLE3_X = 42; // -46 would hit the boundary

    public static double BLUE_SAMPLE1_Y = 30;
    public static double BLUE_SAMPLE2_Y = 23;
    public static double BLUE_SAMPLE3_Y = 23;

    public static double BLUE_SAMPLE1_HEADING = -161;
    public static double RED_SAMPLE2_HEADING = 0;
    public static double RED_SAMPLE3_HEADING = 0;


    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, BLUE_SCORE_START_POSE);
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
        TrajectoryActionBuilder startToBasketTab = drive.actionBuilder(BLUE_SCORE_START_POSE)
                .strafeToLinearHeading(new Vector2d(-38, -56), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(BLUE_BASKET_X, BLUE_BASKET_Y), Math.toRadians(BLUE_BASKET_HEADING));

        TrajectoryActionBuilder driveBasketToSample1Tab = startToBasketTab.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(BLUE_SAMPLE1_X, BLUE_SAMPLE1_Y), Math.toRadians(BLUE_SAMPLE1_HEADING));

        TrajectoryActionBuilder pickUpSample1Tab = driveBasketToSample1Tab.endTrajectory().fresh()
                .setTangent(Math.toRadians(BLUE_SAMPLE1_HEADING))
                .lineToX(BLUE_SAMPLE1_X - 4);

        TrajectoryActionBuilder driveSample1ToBasketTab = pickUpSample1Tab.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(BLUE_BASKET_X, BLUE_BASKET_Y), Math.toRadians(BLUE_BASKET_HEADING));

        TrajectoryActionBuilder driveBasketToSample2Tab = driveSample1ToBasketTab.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(BLUE_SAMPLE2_X, BLUE_SAMPLE2_Y), Math.toRadians(180));

        TrajectoryActionBuilder pickUpSample2Tab = driveBasketToSample2Tab.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(BLUE_SAMPLE2_X - 4, BLUE_SAMPLE2_Y), Math.toRadians(180));

        TrajectoryActionBuilder driveSample2ToBasketTab = pickUpSample2Tab.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(BLUE_BASKET_X, BLUE_BASKET_Y), Math.toRadians(BLUE_BASKET_HEADING));

        TrajectoryActionBuilder driveBasketToSample3Tab = driveSample2ToBasketTab.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(BLUE_SAMPLE3_X, BLUE_SAMPLE3_Y), Math.toRadians(180));

        TrajectoryActionBuilder driveSample3ToBasketTab = driveBasketToSample3Tab.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(BLUE_BASKET_X, BLUE_BASKET_Y), Math.toRadians(BLUE_BASKET_HEADING));

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
                new SequentialAction(
                        arm.armVerticalAction()
                        ,lift.liftDownAction()
                        ,arm.armRobotTravelAction()
                ),
                driveBasketToSample3Tab.build()
        );

        Action cSample1ToBasketAction = new SequentialAction(
                new ParallelAction(
                        driveSample1ToBasketTab.build(),
                        arm.armScoreAction()),
                lift.liftUpAction()
        );
        Action cSample2ToBasketAction = new SequentialAction(
                new ParallelAction(
                        driveSample2ToBasketTab.build(),
                        arm.armScoreAction()),
                lift.liftUpAction()
        );
        Action cSample3ToBasketAction = new ParallelAction(
                driveSample3ToBasketTab.build(),
                arm.armScoreAction(),
                lift.liftUpAction()
        );

        Action cPickUpSample1Action = new ParallelAction(
                pickUpSample1Tab.build(),
                arm.armGroundCollectAction(),
                intake.intakeAction()
        );

        Action cPickUpSample2Action = new ParallelAction(
                pickUpSample2Tab.build(),
                arm.armGroundCollectAction(),
                intake.intakeAction()
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
                        cPickUpSample1Action,
                        arm.armVerticalAction(),
                        cSample1ToBasketAction,
                        new SleepAction(0.1),
                        intake.depositAction(),
                        //
                        cBasketToSample2Action,
                        cPickUpSample2Action,
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



