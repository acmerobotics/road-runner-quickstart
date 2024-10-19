package org.firstinspires.ftc.teamcode.Auto.Paths;



// RR-specific imports
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class Basket2 extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d StartPose1 = new Pose2d(0,0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, StartPose1);

        TrajectoryActionBuilder trajecotryegrig = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-30.03, 14.07), Math.toRadians(0))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-12.4, -30.87), Math.toRadians(-175))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(7, -38), Math.toRadians(143))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-12.4, -38), Math.toRadians(-170))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(7, -38), Math.toRadians(143))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-32.34, -31.87), Math.toRadians(-91))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(7, -38), Math.toRadians(143))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-48, -30), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-46, 0), Math.toRadians(-90));

        Action auto = trajecotryegrig.build();
        waitForStart();
        Actions.runBlocking(auto);
    }
}


