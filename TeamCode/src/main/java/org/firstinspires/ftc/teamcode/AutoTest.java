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
        drive.pose = new Pose2d(-36,-61,Math.toRadians(90));
        Action trajectory =
                drive.actionBuilder(drive.pose)
                        .lineToY(-33)
                        .waitSeconds(1)

                        // Goto Backdrop to place your purple pixel
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-36,-36,Math.toRadians(0)),Math.toRadians(0))
                        .lineToX(48)
                        .waitSeconds(1)

                        // Goto stack and collect 2 white pixels
                        .setReversed(true)
                        .setTangent(Math.toRadians(180))
                        .splineTo(new Vector2d(12, -12),Math.toRadians(180))
                        .lineToX(-61)
                        .waitSeconds(1)

                        // Goto Backstage and drop 2 white pixels
                        .setReversed(false)
                        .lineToX(12)
                        .setTangent(Math.toRadians(0))
                        .splineTo(new Vector2d(48, -36),Math.toRadians(0))
                        .waitSeconds(1)

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
