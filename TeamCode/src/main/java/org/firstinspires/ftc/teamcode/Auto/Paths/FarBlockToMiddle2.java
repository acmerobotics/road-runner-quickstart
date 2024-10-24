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
public class FarBlockToMiddle2 extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d StartPose1 = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap,StartPose1);

        //Pose2d StartPose1 = new Pose2d(-40, -60, 0);
        //drive.setPoseEstimate(StartPose1);

        TrajectoryActionBuilder basket = drive.actionBuilder(StartPose1)
                .strafeTo(new Vector2d(-30.42,-17.84))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-10.04,23.57),-136)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-2.48,9.02),90)
                .waitSeconds(4)
                .strafeTo(new Vector2d(-21.96,26.89))
                .waitSeconds(1)
                .strafeTo(new Vector2d(-2.48,9.02))
                .waitSeconds(4)
                .strafeTo(new Vector2d(-16.96,40.89))
                .waitSeconds(1)
                .strafeTo(new Vector2d(-2.48,9.02))
                .waitSeconds(4)
                .strafeToLinearHeading(new Vector2d(4.5, 46.93),-178);



// RR-specific imports



        Action path = basket.build();
        waitForStart();
        Actions.runBlocking(path);

    }
}
