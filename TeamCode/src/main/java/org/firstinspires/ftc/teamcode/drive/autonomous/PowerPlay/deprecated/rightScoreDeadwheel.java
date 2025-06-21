package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.PowerPlay.deprecated;/*
package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.PowerPlay.deprecated;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Config
@Autonomous(name = "Right Score w/ Deadwheel Adjustments")
public class rightScoreDeadwheel extends LinearOpMode {
    private final Pose2d startPose = new Pose2d(36, -64.25, Math.toRadians(90));
    private final Pose2d scorePose = new Pose2d(41.5, -12, Math.toRadians(145));

    private final double maxSpeed = 45.0, maxAccel = 30.0;

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        TrajectorySequence goToScore = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(scorePose, Math.toRadians(60))
                .build();

        waitForStart();

        drive.followTrajectorySequence(goToScore);
        sleep(100);
    }
}
*/
