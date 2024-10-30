package org.firstinspires.ftc.teamcode.Auto.Paths;



// RR-specific imports
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class BasketBlue extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d StartPose1 = new Pose2d(0,0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, StartPose1);

        TrajectoryActionBuilder trajecotryegrig = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-31.26, 13.07), Math.toRadians(0))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-25.82, -25.72), Math.toRadians(-180))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(8, -39), Math.toRadians(132))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-7.12, -36.617), Math.toRadians(-180))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(8, -39), Math.toRadians(132))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-32.81, -27.32), Math.toRadians(-90))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(8, -39), Math.toRadians(132))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-48, -30), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-48, 0), Math.toRadians(-90));

        Action auto = trajecotryegrig.build();
        waitForStart();
        Actions.runBlocking(auto);
    }
}


