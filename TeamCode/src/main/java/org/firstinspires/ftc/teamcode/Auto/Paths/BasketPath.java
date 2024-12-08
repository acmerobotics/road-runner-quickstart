package org.firstinspires.ftc.teamcode.Auto.Paths;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class BasketPath extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d StartPose1 = new Pose2d(0,0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, StartPose1);

        TrajectoryActionBuilder path = drive.actionBuilder(StartPose1)
                .strafeToLinearHeading(new Vector2d(-15.75, 6.99), Math.toRadians(45))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(-9.95, 8.97), Math.toRadians(90))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(-15.75, 6.99), Math.toRadians(45))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(-15.60, 9.45), Math.toRadians(90))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(-15.75, 6.99), Math.toRadians(45))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(-14.92, 10.38), Math.toRadians(125))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(-15.75, 6.99), Math.toRadians(45))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(-16.31, 52.08), Math.toRadians(-180))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(12.99, 55.52), Math.toRadians(-180));

        Action pathh = path.build();
        waitForStart();
        Actions.runBlocking(pathh);


    }

}
