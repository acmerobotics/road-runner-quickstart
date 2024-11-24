package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Common.Claw;
import org.firstinspires.ftc.teamcode.Common.Fourbar;
import org.firstinspires.ftc.teamcode.Common.Rotation;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "RIGHT PARK", group = "Autonomous")
public class RightPark extends LinearOpMode {
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(10, -63, Math.toRadians(180.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Fourbar fourbar = new Fourbar(hardwareMap);
        Rotation rotation = new Rotation(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToXConstantHeading(-45);

        Action park = tab1.endTrajectory().fresh()
                .lineToXConstantHeading(35)
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.ClawClose());
        Actions.runBlocking(fourbar.FourBarUp());
        Actions.runBlocking(rotation.RotationHorizontal());


        waitForStart();

        if (isStopRequested()) return;

        Action drop = tab1.build();


        Actions.runBlocking(
                new SequentialAction(
                        drop,
                        claw.ClawOpen(),
                        new SleepAction(10),
                        park
                )
        );
    }
}