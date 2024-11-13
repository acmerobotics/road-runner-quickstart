package org.firstinspires.ftc.teamcode.Auto.FinalPathsNoSpeci;



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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.Extendo;
import org.firstinspires.ftc.teamcode.mechanisms.Intaker;
import org.firstinspires.ftc.teamcode.mechanisms.Slides;

@Autonomous
public class BasketRedNoSpeciFinal extends LinearOpMode {
    @Override
    public void runOpMode() {
        Intaker intake = new Intaker(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Slides slides = new Slides(hardwareMap);
        Extendo extendo = new Extendo(hardwareMap);
        Pose2d StartPose1 = new Pose2d(0,0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, StartPose1);

        // start
        TrajectoryActionBuilder basket = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-21.66, 9.5), Math.toRadians(45));
               // .waitSeconds(1);
        TrajectoryActionBuilder firstBlock = basket.fresh()
                .strafeToLinearHeading(new Vector2d(-12.25, 17.75), Math.toRadians(93));
              //  .waitSeconds(1);
//        TrajectoryActionBuilder forwardALil = firstBlock.fresh()
//                .strafeToLinearHeading(new Vector2d(-9.77, 17.43), Math.toRadians(90));
//                //.waitSeconds(1);
        TrajectoryActionBuilder basket2 = firstBlock.fresh()
                .strafeToLinearHeading(new Vector2d(-19.66, 5.5), Math.toRadians(45));
            //    .waitSeconds(1);
        TrajectoryActionBuilder secondBlock = basket2.fresh()
                .strafeToLinearHeading(new Vector2d(-23.77, 17.75), Math.toRadians(90));
            //    .waitSeconds(1);
        TrajectoryActionBuilder basket3 = secondBlock.fresh()
                .strafeToLinearHeading(new Vector2d(-20, 4.5), Math.toRadians(45));
            //    .waitSeconds(1);
        TrajectoryActionBuilder thirdBlock = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-32.81, -27.32), Math.toRadians(-90));
                //.waitSeconds(1);
        TrajectoryActionBuilder parking = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-48, -30), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-48, 0), Math.toRadians(-90));


        Action toBasket = basket.build();
        Action toBasket2 = basket2.build();
        Action toBasket3 = basket3.build();
        //Action foward = forwardALil.build();
        Action block1 = firstBlock.build();
        Action block2 = secondBlock.build();
        Action block3 = thirdBlock.build();
        Action park = parking.build();

        waitForStart();

        /*Actions.runBlocking(new SequentialAction(
                toBasket,
                block1,
                foward
        ));*/

        Actions.runBlocking(new SequentialAction(
                //1+0
                claw.flop(),
                intake.flop(),
                toBasket,
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
                new SleepAction(0.5),
                intake.flip(),
                intake.intake(),
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
                        toBasket2
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
                intake.flip(),
                intake.intake(),
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
                        toBasket3
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

        ));
    }
}



