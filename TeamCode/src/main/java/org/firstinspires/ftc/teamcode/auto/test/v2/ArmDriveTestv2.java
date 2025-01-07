package org.firstinspires.ftc.teamcode.auto.test.v2;




import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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


    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, RED_SCORE_START_POSE);
        Armv2 arm = new Armv2(hardwareMap);
        Liftv2 lift = new Liftv2(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Wristv2 wrist = new Wristv2(hardwareMap);

        Action scoreHighAction2 = new ParallelAction(
                arm.armVerticalAction(),
                lift.liftUpAction(),
                wrist.wristFoldOutAction()
        );

        TrajectoryActionBuilder drivetobasket = drive.actionBuilder(RED_SCORE_START_POSE)
                .strafeToLinearHeading(new Vector2d(-36, -56), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(45));

        Action cStartToBasketScoreAction = new ParallelAction(drivetobasket.build(), scoreHighAction2);


        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking
                ( new SequentialAction(
                        cStartToBasketScoreAction,
                        claw.clawOpenAction()
                ));






        ;
    } // runOpMode



}


