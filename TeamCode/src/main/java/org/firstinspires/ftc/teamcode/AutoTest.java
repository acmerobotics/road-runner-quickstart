package org.firstinspires.ftc.teamcode;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import android.drm.DrmStore;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous (name="Robot: Auto Test--BE CAREFUL!!", group="Robot")
public class AutoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
//        Action traj =
//                drive.actionBuilder(drive.pose)
////                        .lineToY(20)   //TODO: lineTo vs splineTo??
////                        .turn(Math.PI / 2)
//                        .splineTo(new Vector2d(20,20), Math.toRadians(45))
//                        .waitSeconds(1)
//                        .strafeTo(new Vector2d(20,10))
//                        .build();
        //TODO: lineToConstantHeading vs lineTo?


        waitForStart();

            while(!isStopRequested())
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
//                            .lineToY(20)   //TODO: lineTo vs splineTo??
//                            .turn(Math.PI / 2)
                            .splineTo(new Vector2d(20,20), Math.toRadians(45))
                            .waitSeconds(1)
                            .strafeTo(new Vector2d(20,10))
                            .build()
            );
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", drive.pose.heading);
            telemetry.update();

//            runBlocking(traj); //TODO: make brainSTEM robot class




    }
}
