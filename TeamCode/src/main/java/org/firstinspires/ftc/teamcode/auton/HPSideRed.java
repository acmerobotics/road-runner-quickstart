package org.firstinspires.ftc.teamcode.auton;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.KalmanDrive;
import org.firstinspires.ftc.teamcode.subsystems.vision.CVMaster;

@Autonomous(name = "HPSideRed", group = "Autonomous")
public class HPSideRed extends LinearOpMode {
    KalmanDrive drive;
    Limelight3A limelight;
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(12, -60, Math.toRadians(90));
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        drive = new KalmanDrive(hardwareMap, beginPose, limelight);

        Action auton = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(110))
                .splineToLinearHeading(new Pose2d(0, -40, Math.toRadians(90)), Math.toRadians(110))
                .waitSeconds(0.5)

                .setTangent(-Math.PI/10)
                .splineToLinearHeading(new Pose2d(58, -50, Math.toRadians(90)), -Math.PI/10)
                .waitSeconds(0.5)

                .turn(Math.toRadians(20))
                .waitSeconds(0.5)

                .turn(Math.toRadians(-20))
                .waitSeconds(0.5)

                .turn(Math.toRadians(-20))
                .waitSeconds(1)

                // gonna see me cycling
                .setTangent(9 * Math.PI/10)
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(90)), 9 * Math.PI/10)
                .waitSeconds(0.5)
                .setTangent(-Math.PI/10)
                .splineToLinearHeading(new Pose2d(37, -40, Math.toRadians(90)), -Math.PI/10)
                .waitSeconds(0.5)

                .setTangent(9 * Math.PI/10)
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(90)), 9 * Math.PI/10)

                .build();
        telemetry.addData("is","starting");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        auton
                ));
    }
}