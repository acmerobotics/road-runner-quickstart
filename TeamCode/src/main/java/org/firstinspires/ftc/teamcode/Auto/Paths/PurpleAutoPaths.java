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
        Pose2d StartPose1 = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap,StartPose1);


        TrajectoryActionBuilder basket2 = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(16.39,-38), Math.toRadians(0))
                .waitSeconds(1)
                .strafeTo(new Vector2d(16.39,-51))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(38,-30), Math.toRadians(-90))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(10,-30), Math.toRadians(0))
                .waitSeconds(1)
                .lineToX(20)
                .waitSeconds(10)
                .lineToX(5);



        Action path2 = basket2.build();
        waitForStart();
        Actions.runBlocking(path2);

    }
}

