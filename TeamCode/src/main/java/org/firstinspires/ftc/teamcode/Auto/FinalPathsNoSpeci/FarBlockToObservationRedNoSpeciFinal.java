package org.firstinspires.ftc.teamcode.Auto.FinalPathsNoSpeci;



// RR-specific imports
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

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.Extendo;
import org.firstinspires.ftc.teamcode.mechanisms.Intaker;
import org.firstinspires.ftc.teamcode.mechanisms.Slides;

@Autonomous
public class FarBlockToObservationRedNoSpeciFinal extends LinearOpMode {

    @Override
    public void runOpMode() {
        Intaker intake = new Intaker(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Slides slides = new Slides(hardwareMap);
        Extendo extendo = new Extendo(hardwareMap);
        Pose2d StartPose1 = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap,StartPose1);

        //Pose2d StartPose1 = new Pose2d(-40, -60, 0);
        //drive.setPoseEstimate(StartPose1);

        TrajectoryActionBuilder basketStartTraj = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-21.66, 9.5), Math.toRadians(45));
        TrajectoryActionBuilder park = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(0));

        Action basket1 = basketStartTraj.build();
        Action parking = park.build();

        waitForStart();
        Actions.runBlocking(new SequentialAction(
                claw.flop(),
                basket1,
                slides.slideTopBasket(),
                claw.flip(),
                new SleepAction(0.5),
                claw.flop(),
                slides.retract(),
                parking







        ));

    }
}

