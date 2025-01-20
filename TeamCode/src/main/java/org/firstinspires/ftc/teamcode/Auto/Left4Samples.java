package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Common.Claw;
import org.firstinspires.ftc.teamcode.Common.Extension;
import org.firstinspires.ftc.teamcode.Common.Fourbar;
import org.firstinspires.ftc.teamcode.Common.Lift;
import org.firstinspires.ftc.teamcode.Common.Rotation;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Left 4 Samples", group = "Autonomous")
public class Left4Samples extends LinearOpMode {
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-8, -61, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Fourbar fourbar = new Fourbar(hardwareMap);
        Rotation rotation = new Rotation(hardwareMap);
        Extension extension = new Extension(hardwareMap);
        Lift lift = new Lift(hardwareMap);
// auto fix
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-52, -50, Math.toRadians(226)), Math.toRadians(200));

        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(226)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-48, -36, Math.toRadians(80)), Math.toRadians(80));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-47, -40, Math.toRadians(90)))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-52, -50, Math.toRadians(226)), Math.toRadians(270));
        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(226)))
                .setTangent(Math.toRadians(120))
                .splineToLinearHeading(new Pose2d(-60, -40, Math.toRadians(90)), Math.toRadians(90));
        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(-58, -40, Math.toRadians(90)))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-52, -50, Math.toRadians(226)), Math.toRadians(226));
        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(226)))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-30, 10, Math.toRadians(180)), Math.toRadians(0));

        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(fourbar.FourBarUp());
        Actions.runBlocking(claw.ClawClose());
        Actions.runBlocking(lift.resetEncoder());


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
                        lift.SlidesToNet(),
                        new SleepAction(0.5),
                        fourbar.FourBarDown(),
                        new SleepAction(1),
                        claw.ClawOpen(),
                        new SleepAction(0.5),
                        fourbar.FourBarUp(),
                        new SleepAction(1),
                        lift.SlidesDown_new(),
                        pickSample1,
                        fourbar.FourBarDown(),
                        new SleepAction(1),
                        claw.ClawClose(),
                        new SleepAction(0.5),
                        fourbar.FourBarUp(),
                        dropSample1,
                        lift.SlidesToNet(),
                        new SleepAction(0.5),
                        fourbar.FourBarDown(),
                        new SleepAction(0.5),
                        claw.ClawOpen(),
                        new SleepAction(0.5),
                        fourbar.FourBarUp(),
                        new SleepAction(1),
                        lift.SlidesDown_new(),
                        new SleepAction(1),
                        pickSample2,
                        new SleepAction(0.5),
                        dropSample2,
                        new SleepAction(0.5),
                        park
                )
        );
    }
}
