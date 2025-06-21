package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.PowerPlay.demo;/*
package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.PowerPlay.demo;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "demo")
@Disabled
public class asyncInterruptionTest extends LinearOpMode {

    SampleMecanumDrive drive;

    private final double maxAngVel = Math.toRadians(10.0);
    private final double maxAngAccel = Math.toRadians(5.0);

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence turnLeft = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(90), maxAngVel, maxAngAccel)
                .build();

        TrajectorySequence turnRight = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(-90), maxAngVel, maxAngAccel)
                .build();

        TrajectorySequence doNothing = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(0), maxAngVel, maxAngAccel)
                .build();

        waitForStart();

        long startTime = System.currentTimeMillis();

        drive.followTrajectorySequenceAsync(turnLeft);

        while ((System.currentTimeMillis() - startTime) < 5000) {
            drive.update();
        }

        drive.followTrajectorySequenceAsync(doNothing);
        drive.update();
    }
}
*/
