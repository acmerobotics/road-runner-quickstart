package org.firstinspires.ftc.teamcode.AutoArchive.FinalPathsNoSpeci;



// RR-specific imports
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.Extendo;
import org.firstinspires.ftc.teamcode.mechanisms.ExtendoV2;
import org.firstinspires.ftc.teamcode.mechanisms.Intaker;
import org.firstinspires.ftc.teamcode.mechanisms.Slides;
import org.firstinspires.ftc.teamcode.mechanisms.SlidesV2;


@Autonomous
public class BasketRedNoSpeciFinal extends LinearOpMode {
    @Override
    public void runOpMode() {
        Intaker intake = new Intaker(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        SlidesV2 slides = new SlidesV2(hardwareMap, true);
        ExtendoV2 extendo = new ExtendoV2(hardwareMap);
        Pose2d StartPose1 = new Pose2d(0,0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, StartPose1);

        // start
        TrajectoryActionBuilder basketStartTraj = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-21.66, 9.5), Math.toRadians(45));
        // .waitSeconds(1);
        TrajectoryActionBuilder block1Traj = basketStartTraj.fresh()
                .strafeToLinearHeading(new Vector2d(-12.25, 15.5), Math.toRadians(93));
        //  .waitSeconds(1);
        TrajectoryActionBuilder forward1Traj = block1Traj.fresh()
                .strafeToLinearHeading(new Vector2d(-12.25, 19), Math.toRadians(90));
        //.waitSeconds(1);
        TrajectoryActionBuilder basket1Traj = forward1Traj.fresh()
                .strafeToLinearHeading(new Vector2d(-19.66, 5.5), Math.toRadians(45));
        //    .waitSeconds(1);
        TrajectoryActionBuilder block2Traj = basket1Traj.fresh()
                .strafeToLinearHeading(new Vector2d(-23.77, 15.5), Math.toRadians(90));
        //    .waitSeconds(1);
        TrajectoryActionBuilder forward2Traj = block2Traj.fresh()
                .strafeToLinearHeading(new Vector2d(-23.77, 19), Math.toRadians(90));
        //.waitSeconds(1);
        TrajectoryActionBuilder basket2Traj = forward2Traj.fresh()
                .strafeToLinearHeading(new Vector2d(-20, 4.5), Math.toRadians(45));
        //    .waitSeconds(1);
        TrajectoryActionBuilder block3Traj = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-32.81, -27.32), Math.toRadians(-90));
        //.waitSeconds(1);
        TrajectoryActionBuilder parking = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-48, -30), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-48, 0), Math.toRadians(-90));


        Action basketStart = basketStartTraj.build();
        Action basket1 = basket1Traj.build();
        Action basket2 = basket2Traj.build();
        Action foward1 = forward1Traj.build();
        Action foward2 = forward2Traj.build();
        Action block1 = block1Traj.build();
        Action block2 = block2Traj.build();
        Action block3 = block3Traj.build();
        Action park = parking.build();

        waitForStart();

        /*Actions.runBlocking(new SequentialAction(
                toBasket,
                block1,
                foward
        ));*/


        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                //1+0
                claw.flop(),
                intake.flop(),
                basketStart,
                slides.slideTopBasket(),
                claw.flip(),
                new SleepAction(0.5),
                claw.flop(),
                new ParallelAction(
                        slides.retract(),
                        block1
                ),
                //1+1
                //new SleepAction(1),
                extendo.extend(),
                //new SleepAction(0.5),
                intake.upFlip(),
                intake.intake(),
                foward1,
                intake.flip(),
                new SleepAction(1.5),
                intake.flop(),
                intake.creep(),
                extendo.retract(),
                new SleepAction(0.5),
                intake.extake(),
                new SleepAction(1),
                intake.off(),
                new ParallelAction(
                        slides.slideTopBasket(),
                        basket1
                ),
                claw.flip(),
                new SleepAction(0.5),
                claw.flop(),
                new ParallelAction(
                        slides.retract(),
                        //1+2
                        //new SleepAction(1),
                        block2
                ),
                extendo.extend(),
                new SleepAction(0.5),
                intake.upFlip(),
                intake.intake(),
                foward2,
                intake.flip(),
                new SleepAction(1.5),
                intake.flop(),
                intake.creep(),
                extendo.retract(),
                new SleepAction(0.5),
                intake.extake(),
                new SleepAction(1),
                intake.off(),
                new ParallelAction(
                        slides.slideTopBasket(),
                        basket2
                ),
                claw.flip(),
                new SleepAction(0.5),
                claw.flop(),
                new SleepAction(0.5),
                slides.retract(),
                new SleepAction(1)

                /*

                new SleepAction(1),
                intake.off(),
                toBasket,
                slides.slideTopBasket(),
                claw.flip(),
                claw.flop(),
                slides.retract(),
                block2,
                extendo.extend(),
                intake.flip(),
                intake.intake(),
                intake.flop(),
                intake.creep(),
                extendo.retract(),
                intake.extake(),
                new SleepAction(1),
                intake.off(),
                toBasket,
                slides.slideTopBasket(),
                claw.flip(),
                claw.flop(),
                slides.retract(),
                block3,
                extendo.extend(),
                intake.flip(),
                intake.intake(),
                intake.flop(),
                intake.creep(),
                extendo.retract(),
                intake.extake(),
                new SleepAction(1),
                intake.off(),
                toBasket,
                slides.slideTopBasket(),
                claw.flip(),
                claw.flop(),
                slides.retract(),
                park

                */

        ), slides.update()));


    }

}



