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
public class FarBlockToMiddleBlueNoSpeci extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d StartPose1 = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap,StartPose1);

        //Pose2d StartPose1 = new Pose2d(-40, -60, 0);
        //drive.setPoseEstimate(StartPose1);

        TrajectoryActionBuilder basket = drive.actionBuilder(StartPose1)
                //.strafeToLinearHeading(new Vector2d(-29.46,-12.24), Math.toRadians(0))
                //.waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-9.42,41.09), Math.toRadians(180))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-2.48,9.02), Math.toRadians(90))
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(-9.54,51.44), Math.toRadians(180))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-2.48,9.02), Math.toRadians(90))
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(-28.81,38.9), Math.toRadians(90))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-2.48,9.02), Math.toRadians(90))
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(6.85,40), Math.toRadians(180));






// RR-specific imports



        Action path = basket.build();
        waitForStart();
        Actions.runBlocking(path);

    }
}
