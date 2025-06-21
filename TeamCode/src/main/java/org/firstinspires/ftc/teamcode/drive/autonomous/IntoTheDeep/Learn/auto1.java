package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.IntoTheDeep.Learn;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

@Config
@Autonomous(name="auto1", group="competition")
public class auto1 extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-39, -63, Math.toRadians(90)));
        Action trajectoryAction1;

        trajectoryAction1 = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(-48,-63))
                .strafeTo(new Vector2d(63,-63))
                .build();

        waitForStart();

        Actions.runBlocking(
                trajectoryAction1
        );
    }
}