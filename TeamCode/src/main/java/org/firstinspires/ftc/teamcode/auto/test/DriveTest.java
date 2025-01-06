package org.firstinspires.ftc.teamcode.auto.test;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@Config
@Autonomous(name = "\uD83D\uDD34 - drive test", group = "RoadRunner 1.0")
public class DriveTest extends LinearOpMode {


    // Start position red near
    Pose2d RED_SCORE_START_POSE = new Pose2d(-36, -60, Math.toRadians(180));


    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, RED_SCORE_START_POSE);
//       Intake intake = new Intake(hardwareMap);
//        Arm arm = new Arm(hardwareMap);
//        Lift lift = new Lift(hardwareMap);
        TrajectoryActionBuilder traj = drive.actionBuilder(RED_SCORE_START_POSE)
                .strafeToLinearHeading(new Vector2d(-36, -56), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(180+45));


//        Action trajectoryActionCloseOut = traj.fresh()
//                .strafeToLinearHeading(new Vector2d(-36, -12), Math.toRadians(180))
//                .build();

        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(traj.build());


        ;
    } // runOpMode



}


