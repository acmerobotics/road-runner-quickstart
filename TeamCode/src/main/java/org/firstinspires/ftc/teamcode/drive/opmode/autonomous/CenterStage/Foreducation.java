package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.CenterStage;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class Foreducation extends LinearOpMode {



   private SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

   private Pose2d startPose = new Pose2d();
    private Pose2d PoseTwo = new Pose2d();
    private Pose2d PoseThree = new Pose2d();
    private Pose2d endPose = new Pose2d();



    @Override
    public void runOpMode() throws InterruptedException {
        drive.setPoseEstimate(startPose);
       TrajectorySequence move = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(PoseTwo)
               .build();
        drive.setHeight(2000);
        drive.setGrip(false);


    }




}
