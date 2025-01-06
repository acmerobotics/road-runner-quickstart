package org.firstinspires.ftc.teamcode.auto.test.v1;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.v1.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.v1.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.v1.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.v1.Wrist;

@Config
@Autonomous(name = "ScoreTest", group = "Test")
@Disabled
public class ScoreTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        Arm arm = new Arm(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);

        Action scoreHighAction = new ParallelAction(
                arm.armScoreAction(),
                lift.liftUpAction(),
                wrist.wristFoldOutAction()
        );

        Action foldBackAction = new ParallelAction(
                arm.armfoldbackaction(),
                lift.liftDownAction(),
                wrist.wristFoldInAction()
        );

        while(!isStopRequested() && !opModeIsActive()) {
            // Wait for the start signal
        }

        waitForStart();

        if (isStopRequested());

        Actions.runBlocking(
                new SequentialAction(
                        scoreHighAction,
                        new SleepAction(0.5),
                        intake.depositAction(),
                        foldBackAction
                )
        );

    } // runOpMode

}
