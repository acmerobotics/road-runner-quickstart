package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.CenterStage;/*
package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.CenterStage;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name = "lf auto")
@Config
public class AutoV1 extends LinearOpMode {

    //pivate stuff abject calling
    private final Pose2d StartLF = new Pose2d(0,0,Math.toRadians(90));
//    private final Pose2d PurpPos = new Pose2d(10,0,Math.toRadians(90));
    SampleMecanumDrive drive;
    private final double travelSpeed = 45.0, travelAccel = 30.0;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        while (!isStarted()){

            drive.initArm();
        }


        drive.setPoseEstimate(StartLF);


        TrajectorySequence PurpPlace = drive.trajectorySequenceBuilder(new Pose2d(12.30, 60.79, Math.toRadians(270.00)))
                .splineTo(new Vector2d(12.01, 35.16), Math.toRadians(270.00))
                .build();

        drive.followTrajectorySequence(PurpPlace);
        drive.setBothGrip(false);
        sleep(1000);
        TrajectorySequence placeWhite = drive.trajectorySequenceBuilder(new Pose2d(12.01, 36.47, Math.toRadians(270.00)))
                .lineToSplineHeading(new Pose2d(11.43, 50.16, Math.toRadians(0.00)))
                .splineToLinearHeading(new Pose2d(45.79, 38.51, Math.toRadians(0.00)), Math.toRadians(320.00))
                .build();
        drive.followTrajectorySequence(placeWhite);
        drive.setBothGrip(true);
    }
}
*/
