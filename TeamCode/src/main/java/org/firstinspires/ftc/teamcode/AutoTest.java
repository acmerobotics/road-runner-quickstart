package org.firstinspires.ftc.teamcode;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import android.drm.DrmStore;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@Autonomous (name="Robot: Auto Test--BE CAREFUL!!", group="Robot")
public class AutoTest extends ActionOpMode {

    @Override
    public void runOpMode()  {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Action trajectory =
                drive.actionBuilder(drive.pose)
                        .lineToX(20)   //TODO: lineTo vs splineTo??
//                        .turn(Math.PI / 2)
//                        .splineTo(new Vector2d(20, 20), Math.toRadians(45))
//                        .waitSeconds(1)
//                        .strafeTo(new Vector2d(20, 10))
//                        .endTrajectory()
                        .build();

        telemetry.addLine("Trajectory built");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addLine("Inside while loop");
            telemetry.update();

            runBlocking(trajectory);

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", drive.pose.heading);

            telemetry.update();

        }
    }
}
