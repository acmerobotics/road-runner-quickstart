package org.firstinspires.ftc.teamcode.Auto.Paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class BlueAutoPaths extends LinearOpMode {

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d StartPose1 = new Pose2d(40, 60, Math.toRadians(180));
        drive.setPoseEstimate(StartPose1);

        TrajectorySequence basket = drive.trajectorySequenceBuilder(StartPose1)
                .lineToConstantHeading(new Vector2d(0, 36))
                //deposit specimen
                .forward(10)
                .lineToLinearHeading(new Pose2d(-53, -44, Math.toRadians(0)))
                //intake sample
                .lineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(45)))
                //deposit sample
                .lineToLinearHeading(new Pose2d(-55, -44, Math.toRadians(0)))
                //intake sample
                .lineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(45)))
                //deposit sample
                .lineToLinearHeading(new Pose2d(-50, -27, Math.toRadians(90)))
                //intake sample
                .lineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(45)))
                //deposit sample
                .build();


        Pose2d StartPose2 = new Pose2d(20, -60, Math.toRadians(180));
        drive.setPoseEstimate(StartPose2);
        
        TrajectorySequence speciman = drive.trajectorySequenceBuilder(StartPose2)
                .lineToConstantHeading(new Vector2d(5, -25))
                //Drop off specimen
                .forward(8)
                .lineToLinearHeading(new Pose2d(45, -33, Math.toRadians(300)))
                .forward(5)
                //intake block
                .back(5)
                .lineToLinearHeading(new Pose2d(55, -68, Math.toRadians(270)))
                //outtake block into human area
                .back(5)
                .waitSeconds(0.5)
                .strafeLeft(10)
                .lineToLinearHeading(new Pose2d(48, -33, Math.toRadians(300)))
                .forward(5)
                //intake 2nd sample
                .lineToLinearHeading(new Pose2d(50, -68, Math.toRadians(270)))
                //outtake block
                .back(5)
                .waitSeconds(0.5)
                .strafeLeft(0)
                .lineToLinearHeading(new Pose2d(68, -33, Math.toRadians(300)))
                .forward(5)
                //intake 3rd block
                .back(8)
                .lineToLinearHeading(new Pose2d(45, -68, Math.toRadians(270)))
                //outtake block
                .build();

    }
}

