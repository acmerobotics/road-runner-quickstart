package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Common.Claw;
import org.firstinspires.ftc.teamcode.Common.Extension;
import org.firstinspires.ftc.teamcode.Common.Fourbar;
import org.firstinspires.ftc.teamcode.Common.Lift;
import org.firstinspires.ftc.teamcode.Common.Rotation;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Left4Samples extends LinearOpMode {
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-10, -61, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Fourbar fourbar = new Fourbar(hardwareMap);
        Rotation rotation = new Rotation(hardwareMap);
        Extension extension = new Extension(hardwareMap);
        Lift lift = new Lift(hardwareMap);
// auto fix
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(-54, -54, Math.toRadians(226)), Math.toRadians(200));
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(226)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-47, -40, Math.toRadians(90)), Math.toRadians(90));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-47, -40, Math.toRadians(90)))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-54, -54, Math.toRadians(226)), Math.toRadians(226));
        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(226)))
                .setTangent(Math.toRadians(120))
                .splineToLinearHeading(new Pose2d(-58, -40, Math.toRadians(90)), Math.toRadians(90));
        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(-58, -40, Math.toRadians(90)))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-54, -54, Math.toRadians(226)), Math.toRadians(226));
        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(226)))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-30, 10, Math.toRadians(180)), Math.toRadians(0));

        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.ClawOpen());
        Actions.runBlocking(fourbar.FourBarUp());
        Actions.runBlocking(rotation.RotationHorizontal());


        waitForStart();

        if (isStopRequested()) return;

        Action toBasket = tab1.build();
        Action pickSample1 = tab2.build();
        Action dropSample1 = tab3.build();
        Action pickSample2 = tab4.build();
        Action dropSample2 = tab5.build();
        Action park = tab6.build();




        Actions.runBlocking(
                new SequentialAction(
                        toBasket,
                        new SleepAction(2),
                        pickSample1,
                        new SleepAction(2),
                        dropSample1,
                        new SleepAction(2),
                        pickSample2,
                        new SleepAction(2),
                        dropSample2,
                        new SleepAction(2),
                        park
                )
        );
    }
}
