package org.firstinspires.ftc.teamcode.Auto.Vision;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class SpeciTest2 extends LinearOpMode {

    @Override
    public void runOpMode() {


        Pose2d StartPose1 = new Pose2d(0,0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, StartPose1);


        TrajectoryActionBuilder path = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-8, 30), Math.toRadians(180)) //pick up 1
                .waitSeconds(1.5)
                .splineToLinearHeading(new Pose2d(-13.17, 8.59, Math.toRadians(90)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-28.42, -6.61, Math.toRadians(0)), Math.toRadians(180))//2+0
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(-8, 30), Math.toRadians(180)) //pick up 2
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(-28.42, -6.61), Math.toRadians(0)) //3+0
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(-8, 30), Math.toRadians(180)) //pick up 3
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(-28.42, -6.61), Math.toRadians(0)) //4+0
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(-6, 32), Math.toRadians(180)); //park?

        Action pathh = path.build();
        waitForStart();

        Actions.runBlocking(pathh);


    }

}
