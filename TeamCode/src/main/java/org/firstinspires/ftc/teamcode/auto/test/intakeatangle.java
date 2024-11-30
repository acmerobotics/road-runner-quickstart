package org.firstinspires.ftc.teamcode.auto.test;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;


import java.util.List;

@Config
@Autonomous(name = "intakeatangle-driveonly", group = "RoadRunner 1.0")
public class intakeatangle extends LinearOpMode {


    // Start position red near
    Pose2d RED_SCORE_START_POSE = new Pose2d(-38, -60, Math.toRadians(180));


    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, RED_SCORE_START_POSE);
        TrajectoryActionBuilder traj = drive.actionBuilder(RED_SCORE_START_POSE)
                .strafeToLinearHeading(new Vector2d(-38, -56), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-50, -50), Math.toRadians(180+45))
                .strafeToLinearHeading(new Vector2d(-42, -40), Math.toRadians(90+55));
        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;
        Actions.runBlocking(traj.build());

    } // runOpMode



}
