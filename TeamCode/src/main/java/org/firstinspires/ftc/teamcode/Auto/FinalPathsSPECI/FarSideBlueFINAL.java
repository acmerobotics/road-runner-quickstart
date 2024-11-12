package org.firstinspires.ftc.teamcode.Auto.FinalPathsSPECI;



// RR-specific imports
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
public class FarSideBlueFINAL extends LinearOpMode {

    @Override
    public void runOpMode() {

        //Claw claw = new Claw(hardwareMap);
        //Slides slides = new Slides(hardwareMap);

        Pose2d StartPose1 = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap,StartPose1);

        //Pose2d StartPose1 = new Pose2d(-40, -60, 0);
        //drive.setPoseEstimate(StartPose1);

        TrajectoryActionBuilder speciToTopBar = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-20.46,-12.24), Math.toRadians(0))
                .waitSeconds(1);

        TrajectoryActionBuilder backALil = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-22.26, 11.07), Math.toRadians(0))
                .waitSeconds(1);
        TrajectoryActionBuilder pushBlock = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-36,45), Math.toRadians(-90))
                .waitSeconds(1);
        TrajectoryActionBuilder backToObservation = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(8.18,45), Math.toRadians(-90))
                .waitSeconds(1);
        TrajectoryActionBuilder getOUT = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-5, 45), Math.toRadians(180))
                .waitSeconds(1);
        TrajectoryActionBuilder pickUpThatJohn = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-1.2,58), Math.toRadians(-90))
                .waitSeconds(1);
        TrajectoryActionBuilder parking = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(4.85,40), Math.toRadians(180));


        Action speci = speciToTopBar.build();

        Action back = backALil.build();
        Action block1 = pushBlock.build();
        Action toObservation = backToObservation.build();
        Action move = getOUT.build();
        Action specipick = pickUpThatJohn.build();
        Action sleepyJoe = parking.build();



        waitForStart();
        Actions.runBlocking(new SequentialAction(
                //claw.flop(),
                speci,
                //slides.slideTopBar(),
                back,
                //slides.retract(),
                block1,
                toObservation,
                move,
                toObservation,
                //claw.open(),
                specipick,
                //slides.slideWallLevel(),
                //claw.close(),
                //slides.slideBottomBasket(),
                speci,
                //slides.slideTopBar(),
                back,
                //slides.retract(),
                //claw.open(),
                sleepyJoe

        ));

    }
}
