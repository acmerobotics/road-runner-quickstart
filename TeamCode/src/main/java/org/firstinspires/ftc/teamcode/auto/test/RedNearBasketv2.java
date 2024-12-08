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
@Autonomous(name = "\uD83D\uDD34 - RedNearBasketV2", group = "RoadRunner 1.0")
public class RedNearBasketv2 extends LinearOpMode {


    Pose2d RED_SCORE_START_POSE = new Pose2d(-38, -60, Math.toRadians(180));


    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, RED_SCORE_START_POSE);
        Intake intake = new Intake(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);

        TrajectoryActionBuilder traj = drive.actionBuilder(RED_SCORE_START_POSE)
                .strafeToLinearHeading(new Vector2d(-38, -56), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-50, -50), Math.toRadians(180+45));

        Action ta = traj.build();


        Action scoreHighAction = new ParallelAction(
                wrist.wristFoldOutAction(),
                arm.armScoreAction(),
                lift.liftUpAction()
        );

        Action taScore = new ParallelAction(ta, scoreHighAction);

        Action foldBackAction = new ParallelAction(
                arm.armfoldbackaction(),
                lift.liftDownAction()
        );
        Action armpose = new ParallelAction(
                arm.armRobotTravelAction(),
                lift.liftDownAction()
                );
        TrajectoryActionBuilder drivetosample1 = traj.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-30, -22), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-26, -22), Math.toRadians(180));
        Action drivetosample1action = drivetosample1.build();

        Action travelto1 = new ParallelAction(
                armpose,
                drivetosample1action
        );
        Action collectAction = new SequentialAction(
                arm.armGroundCollectAction(),
                intake.intakeAction()
        );
        Action drivetodepositsample = drivetosample1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-50, -50), Math.toRadians(180+45))
                .build();
        Action scoresample1 = new SequentialAction(
                scoreHighAction,
                drivetodepositsample
        );



        while(!isStopRequested() && !opModeIsActive()) {
            // Wait for the start signal
        }

        waitForStart();

        if (isStopRequested());

        Actions.runBlocking(
                new SequentialAction(
                        taScore,
                        new SleepAction(0.1), // sleep for 1 sec
                        intake.depositAction(),
                        travelto1,
                        collectAction,
                        armpose
//                        scoresample1
//                        intake.depositAction(),
//                        foldBackAction
                )
        );

    } // runOpMode



}

