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
public class HPSideRedPreloads extends LinearOpMode {
    Robot robot;
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(12, -60, Math.toRadians(90));
        robot = new Robot(hardwareMap, true);

        Action preloadDrive = robot.drive.actionBuilder(robot.drive.pose)
                .setTangent(Math.toRadians(110))
                .splineToLinearHeading(new Pose2d(0, -40, Math.toRadians(90)), Math.toRadians(110))
                .build();

        Action driveToSpikeR = robot.drive.actionBuilder(new Pose2d(0, -40, Math.toRadians(90)))
                .setTangent(-Math.PI/10)
                .splineToLinearHeading(new Pose2d(58, -45, Math.toRadians(70)), -Math.PI/10)
                .waitSeconds(0.5)
                .build();
        Action driveToObR = robot.drive.actionBuilder(new Pose2d(58, -45, Math.toRadians(70)))
                .lineToYLinearHeading(-51, Math.toRadians(90))
                .build();
        Action driveToSpikeC = robot.drive.actionBuilder(new Pose2d(58, -51, Math.toRadians(90)))
                .lineToYLinearHeading(-47, Math.toRadians(90))
                .build();
        Action driveToObC = robot.drive.actionBuilder(new Pose2d(58, -47, Math.toRadians(90)))
                .lineToYLinearHeading(-51, Math.toRadians(90))
                .build();
        Action driveToSpikeL = robot.drive.actionBuilder(new Pose2d(58, -51, Math.toRadians(90)))
                .lineToYLinearHeading(-45, Math.toRadians(110))
                .build();
        Action driveToObL = robot.drive.actionBuilder(new Pose2d(58, -45, Math.toRadians(110)))
                .lineToYLinearHeading(-51, Math.toRadians(90))
                .build();

        Action driveToObservation1 = robot.drive.actionBuilder(new Pose2d(58, -51, Math.toRadians(90)))
                .lineToYLinearHeading(-43, Math.toRadians(-90))
                .build();
        Action driveToChamber1 = robot.drive.actionBuilder(new Pose2d(58, -43, Math.toRadians(-90)))
                .setTangent(9 * Math.PI/10)
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(90)), 9 * Math.PI/10)
                .build();

        Action driveToObservation2 = robot.drive.actionBuilder(new Pose2d(0, -36, Math.toRadians(90)))
                .setTangent(-Math.PI/10)
                .splineToLinearHeading(new Pose2d(37, -40, Math.toRadians(90)), -Math.PI/10)
                .build();
        Action driveToChamber2 = robot.drive.actionBuilder(new Pose2d(37, -40, Math.toRadians(90)))
                .setTangent(9 * Math.PI/10)
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(90)), 9 * Math.PI/10)
                .build();

        Action driveToObservation3 = robot.drive.actionBuilder(new Pose2d(0, -36, Math.toRadians(90)))
                .setTangent(-Math.PI/10)
                .splineToLinearHeading(new Pose2d(37, -40, Math.toRadians(90)), -Math.PI/10)
                .build();
        Action driveToChamber3 = robot.drive.actionBuilder(new Pose2d(37, -40, Math.toRadians(90)))
                .setTangent(9 * Math.PI/10)
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(90)), 9 * Math.PI/10)
                .build();

        Action park = robot.drive.actionBuilder(new Pose2d(0, -36, Math.toRadians(90)))
                .setTangent(-Math.PI/10)
                .splineToLinearHeading(new Pose2d(37, -40, Math.toRadians(90)), -Math.PI/10)
                .build();

        telemetry.addData("is","starting");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        // PRELOAD DEPOSIT
                        new ParallelAction(
                                preloadDrive,
                                new InstantAction(robot::preloadHighRung)
                        ),
                        robot.outtakeSpecimen(true),

                        // SPIKE RIGHT
                        new ParallelAction(
                                driveToSpikeR,
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        robot.intakePreset(50, true)
                                )
                        ),
                        robot.commands.stopIntake(SampleColors.RED),
                        new ParallelAction(
                                driveToObR,
                                new InstantAction(robot::preloadDropPreset)
                        ),
                        robot.outtakeSample(true),

                       // SPIKE CENTER
                        new ParallelAction(
                                driveToSpikeC,
                                robot.intakePreset(50, true)
                        ),
                        robot.commands.stopIntake(SampleColors.RED),
                        new ParallelAction(
                                driveToObC,
                                new InstantAction(robot::preloadDropPreset)
                        ),
                        robot.outtakeSample(true),

                        //SPIKE LEFT
                        new ParallelAction(
                                driveToSpikeL,
                                robot.intakePreset(50, true)
                        ),
                        robot.commands.stopIntake(SampleColors.RED),
                        new ParallelAction(
                                driveToObL,
                                new InstantAction(robot::preloadDropPreset)
                        ),
                        robot.outtakeSample(true),

                        // **CYCLE #1**
                        driveToObservation1,
                        new InstantAction(() -> robot.intakePreset(50)),
                        robot.commands.stopIntake(SampleColors.RED),
                        new ParallelAction(
                                driveToChamber1,
                                new SequentialAction(
                                        new SleepAction(0),
                                        new InstantAction(robot::highRung)
                                )
                        ),
                        robot.outtakeSpecimen(true),

                        // **CYCLE #2**
                        new ParallelAction(
                                driveToObservation2,
                                new SequentialAction(
                                        new SleepAction(0.1),
                                        robot.intakePreset(50, true)
                                )
                        ),
                        robot.commands.stopIntake(SampleColors.RED),
                        new ParallelAction(
                                driveToChamber2,
                                new SequentialAction(
                                        new SleepAction(0),
                                        new InstantAction(robot::highRung)
                                )
                        ),
                        robot.outtakeSpecimen(true),

                        // **CYCLE #3**
                        new ParallelAction(
                                driveToObservation3,
                                new SequentialAction(
                                        new SleepAction(0.1),
                                        robot.intakePreset(50, true)
                                )
                        ),
                        robot.commands.stopIntake(SampleColors.RED),
                        new ParallelAction(
                                driveToChamber3,
                                new SequentialAction(
                                        new SleepAction(0),
                                        new InstantAction(robot::highRung)
                                )
                        ),
                        robot.outtakeSpecimen(true),

                        // **PARK**
                        new ParallelAction(
                            park,
                            new SequentialAction(
                                new InstantAction(robot::autonObParkPreset)
                            )
                        )
                ));
    }
}