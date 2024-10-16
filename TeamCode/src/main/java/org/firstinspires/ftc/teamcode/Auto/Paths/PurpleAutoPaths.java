package org.firstinspires.ftc.teamcode.Auto.Paths;



// RR-specific imports

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class PurpleAutoPaths extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d StartPose1 = new Pose2d(0, 8, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap,StartPose1);


        TrajectoryActionBuilder basket2 = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(25,-29.5), Math.toRadians(0))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(15, -29.5), Math.toRadians(-180))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(23.24,-46.10), Math.toRadians(-10))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(15, -46.10), Math.toRadians(-180))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(40.54, -30), Math.toRadians(-90))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(40.54, -30), Math.toRadians(-180))
                .waitSeconds(1)
                .strafeTo(new Vector2d(15,-15));

        Action path2 = basket2.build();
        waitForStart();
        Actions.runBlocking(path2);

    }
}

