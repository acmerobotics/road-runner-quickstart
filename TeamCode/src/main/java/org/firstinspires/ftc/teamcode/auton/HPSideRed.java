package org.firstinspires.ftc.teamcode.auton;

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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.KalmanDrive;
import org.firstinspires.ftc.teamcode.subsystems.vision.CVMaster;

import java.util.ArrayList;

@Autonomous(name = "HPSideRed NOT", group = "Autonomous")
public class HPSideRed extends LinearOpMode {
    KalmanDrive drive;
    CVMaster cv;
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(12, -60, Math.toRadians(90));
        cv = new CVMaster(hardwareMap.get(Limelight3A.class, "limelight"), hardwareMap.get(WebcamName.class, "Webcam 1"));
        drive = new KalmanDrive(hardwareMap, beginPose, cv.limelight);
        ArrayList<Pose2d> simPoses = new ArrayList<>();
        simPoses.add(new Pose2d(1, -41, Math.toRadians(90)));
        simPoses.add(new Pose2d(2, -42, Math.toRadians(90)));
        simPoses.add(new Pose2d(3, -40, Math.toRadians(90)));
        simPoses.add(new Pose2d(-1, -40, Math.toRadians(90)));
        simPoses.add(new Pose2d(-2, -40, Math.toRadians(90)));

        Action auton1 = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(110))
                .splineToLinearHeading(new Pose2d(0, -40, Math.toRadians(90)), Math.toRadians(110))
                .waitSeconds(0.5)
                .build();

        Action auton2 = drive.actionBuilder(new Pose2d(0, -40, Math.toRadians(90)))
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
                        auton1
                ));
        Pose2d target = simPoses.get((int) (Math.random()*simPoses.size()));
        telemetry.addData("TARGET POSE", target.toString());
        telemetry.update();
        Action autonDyno = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(110))
                .splineToLinearHeading(target, Math.toRadians(110))
                .waitSeconds(0.5)
                .build();
        Actions.runBlocking(
                new SequentialAction(
                    autonDyno,
                    auton2
                )
        );
    }
}