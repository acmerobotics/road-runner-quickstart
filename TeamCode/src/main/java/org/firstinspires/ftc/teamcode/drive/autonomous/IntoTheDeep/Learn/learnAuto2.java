package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.IntoTheDeep.Learn;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

// This is the second intro to autonomous
// We will be adding claw and lift functionality
// You will find this is actually easier than you'd expect because of RoadRunner

@Config
@Autonomous(name = "learnAuto2", group = "learn")
public class learnAuto2 extends LinearOpMode {
    public Claw claw;
    public Slide slide;

    // Override the runOpMode function from LinearOpMode
    @Override
    public void runOpMode() {
        // Initialize drive variable as a member of MecanumDrive
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-35, -60, Math.toRadians(90)));

        // Initialized claw variable as a member of the Claw class we made above
        claw = new Claw(hardwareMap);
        slide = new Slide(hardwareMap, telemetry);

        // Create action variables
        Action trajectoryAction1;
        Action trajectoryAction2;

        // Define action variables
        trajectoryAction1 = drive.actionBuilder(drive.localizer.getPose())
//                .turn(Math.toRadians(-90))
//                .lineToX(0)
//                .turn(Math.toRadians(90))
//                .lineToY(-34)
//                .turn(Math.toRadians(90))
//                .lineToX(-60)
//                .turn(Math.toRadians(-90))
//                .lineToY(34)
//                .turn(Math.toRadians(-90))
//                .lineToX(0)
//                .turn(Math.toRadians(90))
//                .lineToY(60)
//                .turn(Math.toRadians(90))
//                .lineToX(-35)
//                .turn(Math.toRadians(90))

            .turn(Math.toRadians(-90))
                .lineToX(0)
                .turn(Math.toRadians(90))
                .build();

        trajectoryAction2 = drive.actionBuilder(new Pose2d(60, -35, Math.toRadians(90)))
                .turn(Math.toRadians(90))
                .lineToX(34)
                .turn(Math.toRadians(-90))
                .lineToY(34)
                .turn(Math.toRadians(90))
                .lineToX(60)
                .turn(Math.toRadians(-90))
                .lineToY(-34)
                .turn(Math.toRadians(-90))
                .lineToX(0)
                .turn(Math.toRadians(90))
                .lineToY(-60)
                .turn(Math.toRadians(90))
                .lineToX(35)
                .turn(Math.toRadians(90))
                .build();

        // This stops everything the robot does until the start button is pressed
        waitForStart();

        // This runs our actions
        Actions.runBlocking(
            new SequentialAction(
                new ParallelAction(
                    claw.closeClaw(),
                    trajectoryAction1
                )
            )
        );

    }
}
