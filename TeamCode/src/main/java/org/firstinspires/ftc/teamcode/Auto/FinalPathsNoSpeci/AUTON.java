package org.firstinspires.ftc.teamcode.Auto.FinalPathsNoSpeci;



// RR-specific imports
import com.acmerobotics.dashboard.FtcDashboard;
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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.Extendo;
import org.firstinspires.ftc.teamcode.mechanisms.Intaker;
import org.firstinspires.ftc.teamcode.mechanisms.Slides;

@Autonomous(preselectTeleOp = "BlueTeleop")
public class AUTON extends LinearOpMode {
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
                .strafeToLinearHeading(new Vector2d(-22.5, 9.5), Math.toRadians(45));
        // .waitSeconds(1);
        TrajectoryActionBuilder block1Traj = basketStartTraj.fresh()
                .strafeToLinearHeading(new Vector2d(-12.25, 20), Math.toRadians(95));
        //  .waitSeconds(1);
        TrajectoryActionBuilder basket1Traj = basketStartTraj.fresh()
                .strafeToLinearHeading(new Vector2d(-22.5, 9), Math.toRadians(45));
        //    .waitSeconds(1);
        TrajectoryActionBuilder block2Traj = basket1Traj.fresh()
                .strafeToLinearHeading(new Vector2d(-21.5, 23), Math.toRadians(95));
        //    .waitSeconds(1);
        TrajectoryActionBuilder basket2Traj = basketStartTraj.fresh()
                .strafeToLinearHeading(new Vector2d(-22.5, 9), Math.toRadians(45));
        //    .waitSeconds(1);
        TrajectoryActionBuilder block3Traj = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-32.81, -27.32), Math.toRadians(-90));
        //.waitSeconds(1);
        TrajectoryActionBuilder parking = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-18, 62), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(15, 62), Math.toRadians(180));


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
                new SleepAction(0.45),
                claw.flop(),
                new SleepAction(0.45),
                new ParallelAction(
                        slides.retract(),
                        block1
                ),
                //1+1
                new SleepAction(0.45),
                extendo.mid2(),
                new SleepAction(0.45),
                intake.flip(),
                new SleepAction(0.7),
                intake.intake(),
                extendo.extendBad(),
                new SleepAction(1.25),
                intake.flop(),
                intake.creep(),
                extendo.retract(),
                new SleepAction(0.5),
                intake.extake(),
                new SleepAction(0.45),
                claw.up(),
                new SleepAction(0.2),
                intake.off(),
                new SleepAction(0.25),
                new ParallelAction(
                        slides.slideTopBasket(),
                        basket1
                ),
                claw.flip(),
                new SleepAction(0.45),
                claw.flop(),
                new SleepAction(0.45),
                new ParallelAction(
                        slides.retract(),
                        //1+2
                        //new SleepAction(1),
                        block2
                ),
                new SleepAction(0.45),
                extendo.mid(),
                new SleepAction(0.45),
                intake.flip(),
                new SleepAction(0.70),
                intake.intake(),
                extendo.extendBad(),
                new SleepAction(1.25),
                intake.flop(),
                intake.creep(),
                extendo.retract(),
                new SleepAction(0.5),
                intake.extake(),
                new SleepAction(0.4),
                claw.up(),
                new SleepAction(0.25),
                intake.off(),
                new SleepAction(0.25),
                new ParallelAction(
                        slides.slideTopBasket(),
                        basket2
                ),
                claw.flip(),
                new SleepAction(0.45),
                claw.flop(),
                new SleepAction(0.45),
                slides.retract(),
                new ParallelAction(
                        slides.slideHangLevel(),
                        claw.flop(),
                        extendo.retract(),
                        new SequentialAction(
                                new SleepAction(2),
                                extendo.retract()
                        ),
                        park
                )

        );

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry tele = dashboard.getTelemetry();

        waitForStart();
        Actions.runBlocking(main);
        while (opModeIsActive()) {
            tele.addData("extendo", extendo.getPos());
            tele.update();
            telemetry.addData("extendo encoedr", extendo.getPos());
            telemetry.update();
        }
    }
}



