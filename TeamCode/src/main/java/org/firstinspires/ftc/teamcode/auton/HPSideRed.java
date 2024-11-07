package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.KalmanDrive;
import org.firstinspires.ftc.teamcode.subsystems.vision.CVMaster;
import org.firstinspires.ftc.teamcode.util.enums.SampleColors;
import org.firstinspires.ftc.teamcode.util.misc.FullPose2d;

import java.util.ArrayList;

@Autonomous(name = "HPSideRed NOT", group = "Autonomous")
public class HPSideRed extends LinearOpMode {
    Robot robot;
    CVMaster cv;
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(12, -60, Math.toRadians(90));
//        robot = new KalmanDrive(hardwareMap, beginPose, cv.limelight);
        robot = new Robot(hardwareMap, true);

        Action preloadDrive = robot.drive.actionBuilder(robot.drive.pose)
                .setTangent(Math.toRadians(110))
                .splineToLinearHeading(new Pose2d(0, -40, Math.toRadians(90)), Math.toRadians(110))
                .build();

        Action auton2 = robot.drive.actionBuilder(new Pose2d(0, -40, Math.toRadians(90)))
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
                        new ParallelAction(
                                preloadDrive,
                                new InstantAction(robot::preloadHighRung)
                        ),
                        new InstantAction(robot::outtakeSpecimen),
                        robot.commands.locateTargetsCV(CVMaster.EOCVPipeline.RED_SAMPLE)
                ));

        Pose3D target = robot.cv.findOptimalTarget(robot.drive.pose);
        FullPose2d robotCapturePose = robot.cv.calculateRobotFullPose(target, target.getPosition().x, robot.drive.pose.position.y);
        telemetry.addData("TARGET POSE", target.toString());
        telemetry.update();

        Action intakeAdjustment = robot.drive.actionBuilder(robot.drive.pose)
                .setTangent(Math.toRadians(110))
                .splineToLinearHeading(robotCapturePose.getRobotPose(), Math.toRadians(110))
                .build();

        Actions.runBlocking(
                new SequentialAction(
                    new ParallelAction(
                            intakeAdjustment,
                            new InstantAction(() -> robot.intakePreset(robotCapturePose.intakeExtension))
                    ),
                    robot.commands.stopIntake(SampleColors.RED),
                    new SleepAction(1),
                    auton2
                )
        );
    }
}