package org.firstinspires.ftc.teamcode.Auto.FinalPathsNoSpeci;



// RR-specific imports
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class BasketBlueNoSpeciFinal extends LinearOpMode {
    @Override
    public void runOpMode() {
        Intaker intake = new Intaker(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Slides slides = new Slides(hardwareMap);
        Extendo extendo = new Extendo(hardwareMap);
        Pose2d StartPose1 = new Pose2d(0,0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, StartPose1);

        // start
        TrajectoryActionBuilder basketStartTraj = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-21.66, 9.5), Math.toRadians(45));
        // .waitSeconds(1);
        TrajectoryActionBuilder block1Traj = basketStartTraj.fresh()
                .strafeToLinearHeading(new Vector2d(-13.75, 21), Math.toRadians(90));
        //  .waitSeconds(1);
        TrajectoryActionBuilder basket1Traj = basketStartTraj.fresh()
                .strafeToLinearHeading(new Vector2d(-21.66, 9.5), Math.toRadians(45));
        //    .waitSeconds(1);
        TrajectoryActionBuilder block2Traj = basket1Traj.fresh()
                .strafeToLinearHeading(new Vector2d(-21.77, 21), Math.toRadians(90));
        //    .waitSeconds(1);
        TrajectoryActionBuilder basket2Traj = basketStartTraj.fresh()
                .strafeToLinearHeading(new Vector2d(-21.66, 9.5), Math.toRadians(45));
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

        Action main = new SequentialAction(
                //1+0
                claw.flop(),
                intake.flop(),
                basketStart,
                slides.slideTopBasket(),
                claw.flip(),
                new SleepAction(0.5),
                claw.flop(),
                new SleepAction(0.5),
                new ParallelAction(
                        slides.retract(),
                        block1
                ),
                //1+1
                new SleepAction(1),
                extendo.mid(),
                intake.flip(),
                new SleepAction(1),
                intake.intake(),
                extendo.extendBad(),
                new SleepAction(1.5),
                intake.flop(),
                intake.creep(),
                extendo.retract(),
                new SleepAction(0.5),
                intake.extake(),
                new SleepAction(1.5),
                intake.off(),
                new SleepAction(0.5),
                claw.up(),
                new SleepAction(0.25),
                claw.flop(),
                new ParallelAction(
                        slides.slideTopBasket(),
                        basket1
                ),
                claw.flip(),
                new SleepAction(0.5),
                claw.flop(),
                new SleepAction(0.5),
                new ParallelAction(
                        slides.retract(),
                        //1+2
                        //new SleepAction(1),
                        block2
                ),
                new SleepAction(1),
                extendo.mid(),
                intake.flip(),
                new SleepAction(1),
                intake.intake(),
                extendo.extendBad(),
                new SleepAction(1.5),
                intake.flop(),
                intake.creep(),
                extendo.retract(),
                new SleepAction(0.5),
                intake.extake(),
                new SleepAction(1.5),
                intake.off(),
                new SleepAction(0.5),
                claw.up(),
                new SleepAction(0.25),
                claw.flop(),
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

        );

        Actions.runBlocking(main);
//        TelemetryPacket packet = new TelemetryPacket();
//
//        while (opModeIsActive()) {
//            if (!main.run(packet)) {
//                break;
//            }
//            if (extendo.pid) {
//                extendo.updateMotor();
//            }
//            telemetry.addData("extendo target", extendo.extendoMotorPID.getTargetPosition());
//            telemetry.update();
//        }
    }
}



