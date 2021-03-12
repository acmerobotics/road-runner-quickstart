package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "drive")
public class TrajectorySequenceTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(40)
                .addTemporalMarker(() -> Log.i("temporal marker", "temporal marker 0 called"))
                .turn(Math.toRadians(90))
                .addTemporalMarker(() -> Log.i("temporal marker", "temporal marker 1 called"))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> Log.i("temporal marker", "temporal marker 2 called"))
                .waitSeconds(2)
                .addTemporalMarker(() -> Log.i("temporal marker", "temporal marker 3 called"))
                .splineTo(new Vector2d(60, 30), 0)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(trajectorySequence);

        while(!isStopRequested());
    }
}
