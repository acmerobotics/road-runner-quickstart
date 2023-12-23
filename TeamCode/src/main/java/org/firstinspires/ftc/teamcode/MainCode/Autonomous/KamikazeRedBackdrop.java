package org.firstinspires.ftc.teamcode.MainCode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Config
@TeleOp(name="redkamikaze", group="Linear Opmode")

public final class KamikazeRedBackdrop extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        Pose2d startingPose;
        Pose2d nextPose;
        MecanumDrive drive;

        waitForStart();
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) { //BackstageRed
            startingPose = new Pose2d(12, -64, Math.PI / 2);
            drive = new MecanumDrive(hardwareMap, startingPose);

            nextPose = new Pose2d(64, -64, 0);
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .splineToConstantHeading(nextPose.position, nextPose.heading)
                            .build());
        }
    }
}
