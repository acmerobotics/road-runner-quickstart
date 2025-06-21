package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.PowerPlay.deprecated;/*
package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.PowerPlay.deprecated;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Config
@Autonomous(group = "Competition")
public class BasicAutonomousMoveLeft extends LinearOpMode {
    public static int DISTANCE = 22;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory forward = drive.trajectoryBuilder(new Pose2d()).forward(DISTANCE).build();
        Trajectory left = drive.trajectoryBuilder(new Pose2d()).strafeLeft(5).build();

        waitForStart();

        drive.followTrajectory(left);
        drive.followTrajectory(forward);
    }
}
*/
