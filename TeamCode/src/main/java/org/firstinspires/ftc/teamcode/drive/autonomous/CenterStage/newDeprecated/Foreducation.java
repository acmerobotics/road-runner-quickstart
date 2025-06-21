package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.CenterStage.newDeprecated;/*
package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.CenterStage.newDeprecated;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class Foreducation extends LinearOpMode {

//This class was shown at SOAR it is a simple OpMode I (graeme miracle) wanted to write something that moved the robot

   private SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

   private Pose2d startPose = new Pose2d(0,0,Math.toRadians(0));
    private Pose2d PoseTwo = new Pose2d(10,10,Math.toRadians(180));
    private Pose2d PoseThree = new Pose2d(10,0,Math.toRadians(-90));
    private Pose2d endPose = new Pose2d(20,20,Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException {
        drive.setPoseEstimate(startPose);
       TrajectorySequence move1 = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(PoseTwo)
               .build();
        drive.setHeight(2000);
        drive.setBothGrip(false);
        drive.followTrajectorySequence(move1);

        TrajectorySequence move2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(PoseTwo)
                .build();
        drive.followTrajectorySequence(move2);
        TrajectorySequence move3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(PoseThree)
                .build();
        drive.followTrajectorySequence(move3);
        TrajectorySequence endmove = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(endPose)
                .build();
        drive.followTrajectorySequence(endmove);




    }




}
*/
