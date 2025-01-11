
package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.ExtendoV2;
import org.firstinspires.ftc.teamcode.mechanisms.Intaker;
import org.firstinspires.ftc.teamcode.mechanisms.SlidesV2;

@Autonomous
public class SpeciAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d StartPose1 = new Pose2d(0,0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, StartPose1);
        SlidesV2 slides = new SlidesV2(hardwareMap, true);
        ExtendoV2 extendo = new ExtendoV2(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Intaker intake = new Intaker(hardwareMap);

        Action firstSpeciDropOff = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-28.42, -6.61), Math.toRadians(0)) //1+0
                .waitSeconds(1.5)
                .build();
        Action block1 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-21.85, 18.47), Math.toRadians(125)) //block 1
                .waitSeconds(1.5)
                .build();
        Action deposit1 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-21.06, 21.53), Math.toRadians(45)) //deposit
                .waitSeconds(1.5)
                .build();
        Action block2 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-22.65, 27.62), Math.toRadians(125)) //block 2
                .waitSeconds(1.5)
                .build();
        Action deposit2 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-21.07, 27.18), Math.toRadians(42)) //deposit2
                .waitSeconds(1.5)
                .build();
        Action pickUp1 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-8, 30), Math.toRadians(180)) //pick up 1
                .waitSeconds(1.5)
                .build();
        Action secondSpeciDropOff = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-28.42, -6.61), Math.toRadians(0)) //2+0
                .waitSeconds(1.5)
                .build();
         Action pickUp2 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-8, 30), Math.toRadians(180)) //pick up 2
                .waitSeconds(1.5)
                .build();
        Action thirdSpeciDropOff = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-28.42, -6.61), Math.toRadians(0)) //3+0
                .waitSeconds(1.5)
                .build();
        Action pickUp3 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-8, 30), Math.toRadians(180)) //pick up 3
                .waitSeconds(1.5)
                .build();
        Action fourthSpeciDropOff = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-28.42, -6.61), Math.toRadians(0)) //4+0
                .waitSeconds(1.5)
                .build();
        Action park = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-6, 32), Math.toRadians(180))
                .build(); //park?


        waitForStart();

        Actions.runBlocking(new SequentialAction(
                firstSpeciDropOff,
                slides.slideTopBar(),
                slides.slideTopBarClip(),
                new SleepAction(0.25),
                claw.open(),
                new SleepAction(0.2),
                slides.retract(),
                new ParallelAction(
                        block1,
                        extendo.extend()
                ),
                intake.flat(),
                deposit1,
                block2,
                deposit2,
                new ParallelAction(
                    extendo.retract(),
                    intake.flop(),
                    pickUp1
                ),
                claw.close(),
                new ParallelAction(
                        slides.slideTopBar(),
                        secondSpeciDropOff
                ),
                slides.slideTopBarClip(),
                new SleepAction(0.25),
                claw.open(),
                new SleepAction(0.2),
                slides.retract(),
                pickUp2,
                claw.close(),
                new ParallelAction(
                        slides.slideTopBar(),
                        thirdSpeciDropOff
                ),
                slides.slideTopBarClip(),
                //new SleepAction(0.25),
                claw.open(),
                //new SleepAction(0.2),
                slides.retract(),
                pickUp3,
                claw.close(),
                new ParallelAction(
                        slides.slideTopBar(),
                        fourthSpeciDropOff
                ),
                slides.slideTopBarClip(),
                //new SleepAction(0.25),
                claw.open(),
                //new SleepAction(0.2),
                slides.retract(),
                park

        ));


    }
}
