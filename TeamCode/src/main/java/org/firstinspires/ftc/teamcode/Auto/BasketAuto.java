package org.firstinspires.ftc.teamcode.Auto;

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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.ExtendoV2;
import org.firstinspires.ftc.teamcode.mechanisms.Intaker;
import org.firstinspires.ftc.teamcode.mechanisms.SlidesV2;

@Config
@Autonomous
public class BasketAuto extends LinearOpMode {


    @Override
    public void runOpMode() {

        Pose2d StartPose1 = new Pose2d(0,0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, StartPose1);
        SlidesV2 slides = new SlidesV2(hardwareMap, true);
        ExtendoV2 extendo = new ExtendoV2(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Intaker intake = new Intaker(hardwareMap);

        TrajectoryActionBuilder path = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-15.75, 6.99), Math.toRadians(45))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(-9.95, 8.97), Math.toRadians(90))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(-15.75, 6.99), Math.toRadians(45))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(-15.60, 9.45), Math.toRadians(90))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(-15.75, 6.99), Math.toRadians(45))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(-14.92, 10.38), Math.toRadians(125))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(-15.75, 6.99), Math.toRadians(45))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(-16.31, 52.08), Math.toRadians(-180))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(12.99, 55.52), Math.toRadians(-180));


        TrajectoryActionBuilder depositPreloadT = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-15.75, 6.99), Math.toRadians(45));
        TrajectoryActionBuilder firstSampleT = depositPreloadT.fresh()
                .strafeToLinearHeading(new Vector2d(-15.75, 6.99), Math.toRadians(90));
        TrajectoryActionBuilder firstSampleSweepT = firstSampleT.fresh()
                .strafeToLinearHeading(new Vector2d(-9.95, 11.97), Math.toRadians(90));
        TrajectoryActionBuilder depositFirstT = firstSampleSweepT.fresh()
                .strafeToLinearHeading(new Vector2d(-15.75, 6.99), Math.toRadians(45));
        TrajectoryActionBuilder secondSampleT = depositFirstT.fresh()
                .strafeToLinearHeading(new Vector2d(-15.60, 9.45), Math.toRadians(90));
        TrajectoryActionBuilder secondSampleSweepT = secondSampleT.fresh()
                .strafeToLinearHeading(new Vector2d(-15.60, 12.45), Math.toRadians(90));
        TrajectoryActionBuilder depositSecondT = secondSampleSweepT.fresh()
                .strafeToLinearHeading(new Vector2d(-15.75, 6.99), Math.toRadians(45));
        TrajectoryActionBuilder thirdSampleT = depositSecondT.fresh()
                .strafeToLinearHeading(new Vector2d(-14.92, 10.38), Math.toRadians(125));
        TrajectoryActionBuilder thirdSampleSweepT = thirdSampleT.fresh()
                .strafeToLinearHeading(new Vector2d(-14.92, 13.38), Math.toRadians(125));
        TrajectoryActionBuilder depositThirdT = thirdSampleSweepT.fresh()
                .strafeToLinearHeading(new Vector2d(-15.75, 6.99), Math.toRadians(45));
        TrajectoryActionBuilder parkT = depositThirdT.fresh()
                .strafeToLinearHeading(new Vector2d(-16.31, 52.08), Math.toRadians(-180))
                .strafeToLinearHeading(new Vector2d(12.99, 55.52), Math.toRadians(-180));

        Action depositPreload = depositPreloadT.build();
        Action firstSample = firstSampleT.build();
        Action firstSampleSweep = firstSampleSweepT.build();
        Action depositFirst = depositFirstT.build();
        Action secondSample = secondSampleT.build();
        Action secondSampleSweep = secondSampleSweepT.build();
        Action depositSecond = depositSecondT.build();
        Action thirdSample = thirdSampleT.build();
        Action thirdSampleSweep = thirdSampleSweepT.build();
        Action depositThird = depositThirdT.build();
        Action park = parkT.build();


        waitForStart();
        Actions.runBlocking(new SequentialAction(
                depositPreload,
                firstSample,
                firstSampleSweep,
                depositFirst
//                //preload
//                claw.flop(),
//                intake.flop(),
//                extendo.retract(),
//                new ParallelAction(
//                        depositPreload
//                        slides.slideTopBasket(),
//                        extendo.balance()
//                ),
//                claw.flip(),
//                new SleepAction(0.7),
//                claw.flop(),
//                //0-1
//                firstSample
//                new ParallelAction(
//                        slides.retract(),
//                        extendo.extend(),
//                        intake.flip(),
//                        intake.intake()
//                )
//                firstSampleSweep,
//                new ParallelAction(
//                        intake.creep(),
//                        extendo.retract()
//                ),
//                depositFirst,
//                intake.extake(),
//                new SleepAction(0.7),
//                claw.up(),
//                new ParallelAction(
//                        slides.slideTopBasket(),
//                        extendo.balance()
//                ),
//                claw.flip(),
//                new SleepAction(0.7),
//                claw.flop(),
//                //0-2
//                new ParallelAction(
//                        secondSample,
//                        slides.retract(),
//                        extendo.extend(),
//                        intake.flip(),
//                        intake.intake()
//                ),
//                secondSampleSweep,
//                new ParallelAction(
//                        intake.creep(),
//                        extendo.retract()
//                ),
//                depositSecond,
//                intake.extake(),
//                new SleepAction(0.7),
//                claw.up(),
//                new ParallelAction(
//                        slides.slideTopBasket(),
//                        extendo.balance()
//                ),
//                claw.flip(),
//                new SleepAction(0.7),
//                claw.flop(),
//                //0-3
//                new ParallelAction(
//                        thirdSample,
//                        slides.retract(),
//                        extendo.extend(),
//                        intake.flip(),
//                        intake.intake()
//                ),
//                thirdSampleSweep,
//                new ParallelAction(
//                        intake.creep(),
//                        extendo.retract()
//                ),
//                depositThird,
//                intake.extake(),
//                new SleepAction(0.7),
//                claw.up(),
//                new ParallelAction(
//                        slides.slideTopBasket()
//                ),
//                claw.flip(),
//                new SleepAction(0.7),
//                claw.flop(),
//                new ParallelAction(
//                        park,
//                        slides.slideBottomBar()
//                )
        ));


    }

}
