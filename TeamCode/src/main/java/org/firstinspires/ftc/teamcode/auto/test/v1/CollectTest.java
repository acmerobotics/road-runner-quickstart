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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.v1.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.v1.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.v1.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.v1.Wrist;



@Autonomous(name = "collecttest", group = "RoadRunner 1.0")
@Disabled
public class CollectTest extends LinearOpMode {


    // Start position red near
    Pose2d RED_SCORE_START_POSE = new Pose2d(0, 0, Math.toRadians(0));


    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, RED_SCORE_START_POSE);
        Intake intake = new Intake(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        TrajectoryActionBuilder traj = drive.actionBuilder(RED_SCORE_START_POSE)
                .strafeToLinearHeading(new Vector2d(4, 0), Math.toRadians(0));


        Action intakeAction = new ParallelAction(
                traj.build(),
                intake.intakeAction()
        );

        while(!isStopRequested() && !opModeIsActive()) {
        }

        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        wrist.wristFoldOutAction(),
                        arm.armGroundCollectAction(),
                        intakeAction,
                        new SleepAction(5),
                        arm.armRobotTravelAction(),
                        new SleepAction(2),
                        arm.armfoldbackaction()
                )
        );






    } // runOpMode



}

