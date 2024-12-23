package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Common.Claw;
import org.firstinspires.ftc.teamcode.Common.Extension;
import org.firstinspires.ftc.teamcode.Common.Fourbar;
import org.firstinspires.ftc.teamcode.Common.Lift;
import org.firstinspires.ftc.teamcode.Common.Rotation;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "Auto Select Menu", group = "Autonomous")
public class AutoSelectMenu extends LinearOpMode {
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-10, -63, Math.toRadians(90.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Fourbar fourbar = new Fourbar(hardwareMap);
        Rotation rotation = new Rotation(hardwareMap);
        Extension extension = new Extension(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(0, -37), Math.toRadians(90));

        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.ClawOpen());
        Actions.runBlocking(fourbar.FourBarUp());
        Actions.runBlocking(rotation.RotationHorizontal());

//auto fix
        waitForStart();

        if (isStopRequested()) return;

        Action toChambers = tab1.build();


        Actions.runBlocking(
                new SequentialAction(
//                        toChambers,
                        claw.ClawClose(),
                        new SleepAction(10),
                        fourbar.FourBarDown(),
                        lift.SlidesToBar(),
                        new SleepAction(30)
                )
        );
    }
}