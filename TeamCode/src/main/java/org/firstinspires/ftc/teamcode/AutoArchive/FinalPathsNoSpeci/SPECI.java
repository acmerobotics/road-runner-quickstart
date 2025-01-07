package org.firstinspires.ftc.teamcode.AutoArchive.FinalPathsNoSpeci;



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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.Extendo;
import org.firstinspires.ftc.teamcode.mechanisms.Intaker;
import org.firstinspires.ftc.teamcode.mechanisms.Slides;
@Disabled
@Autonomous(preselectTeleOp = "BlueTeleop")
public class SPECI extends LinearOpMode {
    @Override
    public void runOpMode() {
        Intaker intake = new Intaker(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Slides slides = new Slides(hardwareMap);
        Extendo extendo = new Extendo(hardwareMap);
        Pose2d StartPose1 = new Pose2d(0,0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, StartPose1);

        // start
        TrajectoryActionBuilder lowRung = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-28.42, -6.61), Math.toRadians(0)) //1+0
                .waitSeconds(1.5);
        TrajectoryActionBuilder backALil = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-30.42, -6.61), Math.toRadians(0))
                .waitSeconds(1.5);
        TrajectoryActionBuilder block1Traj = lowRung.fresh()
                .strafeToLinearHeading(new Vector2d(-21.85, 18.47), Math.toRadians(125)) //block 1
                .waitSeconds(1.5);
        TrajectoryActionBuilder deposit = block1Traj.fresh()
                .strafeToLinearHeading(new Vector2d(-21.06, 21.53), Math.toRadians(45)) //deposit
                .waitSeconds(1.5);
        TrajectoryActionBuilder block2Traj = deposit.fresh()
                .strafeToLinearHeading(new Vector2d(-22.65, 27.62), Math.toRadians(125)) //block 2
                .waitSeconds(1.5);
        TrajectoryActionBuilder deposit2 = block2Traj.fresh()
                .strafeToLinearHeading(new Vector2d(-21.07, 27.18), Math.toRadians(42)) //deposit2
               .waitSeconds(1.5);
        TrajectoryActionBuilder pickUp = deposit2.fresh()
                .strafeToLinearHeading(new Vector2d(-8, 30), Math.toRadians(180)) //pick up 1
                .waitSeconds(1.5);
        TrajectoryActionBuilder parking = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-6, 32), Math.toRadians(180)); //park?


        Action drop = lowRung.build();
        Action block1 = block1Traj.build();
        Action block2 = block2Traj.build();
        Action observation = deposit.build();
        Action observation2 = deposit2.build();
        Action pick = pickUp.build();
        Action park = parking.build();
        Action back = backALil.build();

        waitForStart();

        /*Actions.runBlocking(new SequentialAction(
                toBasket,
                block1,
                foward
        ));*/

        Action main = new SequentialAction(

                claw.close(),
                new ParallelAction(
                        drop,
                        slides.slideTopBar()
                ),
                back,
                slides.middle(),
                claw.open(),
                slides.retract(),
                block1,
                extendo.extend(),
                intake.flip(),
                intake.intake(),
                new SleepAction(1.5),
                intake.off(),
                intake.upFlip(),
                observation,
                intake.flip(),
                new SleepAction(1),
                intake.extake(),
                new SleepAction(1),
                intake.off(),
                intake.upFlip(),
                block2,
                intake.flip(),
                intake.intake(),
                new SleepAction(1.5),
                intake.off(),
                intake.upFlip(),
                observation2,
                intake.flip(),
                new SleepAction(1),
                intake.extake(),
                new SleepAction(1),
                intake.off(),
                intake.flop(),
                extendo.retract(),
                new SleepAction(3),
                new ParallelAction(
                    pick,
                    slides.slideWallLevel()
                ),
                claw.close(),
                new ParallelAction(
                        drop,
                        slides.slideTopBar()
                ),
                back,
                slides.middle(),
                claw.open(),
                new ParallelAction(
                        pick,
                        slides.slideWallLevel()
                ),
                claw.close(),
                new ParallelAction(
                        drop,
                        slides.slideTopBar()
                ),
                back,
                slides.middle(),
                claw.open(),
                new ParallelAction(
                        pick,
                        slides.slideWallLevel()
                ),
                claw.close(),
                new ParallelAction(
                        drop,
                        slides.slideTopBar()
                ),
                back,
                slides.middle(),
                claw.open(),
                slides.retract(),
                park
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



