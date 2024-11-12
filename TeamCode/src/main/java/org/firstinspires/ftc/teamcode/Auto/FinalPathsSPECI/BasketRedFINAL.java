package org.firstinspires.ftc.teamcode.Auto.FinalPathsSPECI;



// RR-specific imports
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.Extendo;
import org.firstinspires.ftc.teamcode.mechanisms.Intaker;
import org.firstinspires.ftc.teamcode.mechanisms.Slides;

@Autonomous
public class BasketRedFINAL extends LinearOpMode {
    Slides slides = new Slides(hardwareMap);
    Claw claw = new Claw(hardwareMap);
    Intaker intake = new Intaker(hardwareMap);
    Extendo extendo = new Extendo(hardwareMap);

    @Override
    public void runOpMode() {

        Pose2d StartPose1 = new Pose2d(0,0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, StartPose1);


        TrajectoryActionBuilder Model = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-31.26, 13.07), Math.toRadians(0))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-25.82, -25.72), Math.toRadians(-180))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(8, -39), Math.toRadians(132))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-7.12, -36.617), Math.toRadians(-180))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(8, -39), Math.toRadians(132))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-32.81, -27.32), Math.toRadians(-90))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(8, -39), Math.toRadians(132))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-48, -30), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-48, 0), Math.toRadians(-90));

        TrajectoryActionBuilder speciToTopBar = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-31.26, 13.07), Math.toRadians(0))
                .waitSeconds(1);
        TrajectoryActionBuilder backALil = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-31.26, 11.07), Math.toRadians(0))
                .waitSeconds(1);
        TrajectoryActionBuilder firstBlock = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-25.82, -25.72), Math.toRadians(-180))
                .waitSeconds(1);
        TrajectoryActionBuilder basket = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(8, -39), Math.toRadians(132))
                .waitSeconds(1);
        TrajectoryActionBuilder secondBlock = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-7.12, -36.617), Math.toRadians(-180))
                .waitSeconds(1);
        TrajectoryActionBuilder thirdBlock = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-32.81, -27.32), Math.toRadians(-90))
                .waitSeconds(1);
        TrajectoryActionBuilder parking = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-48, -30), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-48, 0), Math.toRadians(-90));

        Action speci = speciToTopBar.build();
        Action back = backALil.build();
        Action block1 = firstBlock.build();
        Action toBasket = basket.build();
        Action block2 = secondBlock.build();
        Action block3 = thirdBlock.build();
        Action park = parking.build();


        waitForStart();
        Actions.runBlocking(new SequentialAction(
                claw.flop(),
                speci,
                slides.slideTopBar(),
                back,
                slides.retract(),
                block1,
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
        ));




    }
}


