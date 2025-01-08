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
    Pose2d RED_SCORE_START_POSE = new Pose2d(-36, -60, Math.toRadians(0));

    Pose2d RED_NEAR_BASKET_POSE = new Pose2d(-54, -54, Math.toRadians(45));




    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, RED_SCORE_START_POSE);
        Armv2 arm = new Armv2(hardwareMap);
        Liftv2 lift = new Liftv2(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Wristv2 wrist = new Wristv2(hardwareMap);

//        Action scoreHighAction2 = new ParallelAction(
//                arm.armVerticalAction(),
//                wrist.wristFoldOutAction()
//        );

        Action scoreHighAction3 = new ParallelAction(
                arm.armScoreAction(),
                lift.liftUpAction()
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
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(45));
        TrajectoryActionBuilder drivetosample1 = drive.actionBuilder(RED_NEAR_BASKET_POSE)
                .strafeToLinearHeading(new Vector2d(-48, -43), Math.toRadians(90));
        TrajectoryActionBuilder drivetosample2 = drive.actionBuilder(RED_NEAR_BASKET_POSE)
                .strafeToLinearHeading(new Vector2d(-60, -40), Math.toRadians(90));
        TrajectoryActionBuilder drivetosample3 = drive.actionBuilder(RED_NEAR_BASKET_POSE)
                .strafeToLinearHeading(new Vector2d(-60, -24), Math.toRadians(180));


        // test only come back at the end
        TrajectoryActionBuilder goBackToStart = drivetobasket.endTrajectory().fresh()
                .strafeToLinearHeading(RED_SCORE_START_POSE.position, 0);
        Action cGoBackToStartAction = new ParallelAction(
                goBackToStart.build(),
                claw.clawOpenAction(),
                arm.armResetAction(),
                wrist.wristFoldInAction());
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



        Actions.runBlocking
                ( new SequentialAction(
                        cStartToBasketScoreAction3,
                        wrist.wristFoldOutAction(),
                        new SleepAction(2),
                        claw.clawOpenAction(),
                        new SleepAction(.2),
                        comedownAction,
                        cGoBackToStartAction

                ));



        ;
    } // runOpMode



}


