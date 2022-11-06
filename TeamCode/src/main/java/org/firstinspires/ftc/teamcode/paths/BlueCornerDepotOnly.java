package org.firstinspires.ftc.teamcode.paths;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "BlueCornerDepotOnly", group = "Auto")
public class BlueCornerDepotOnly extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        final SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        final Pose2d initialPose = new Pose2d(-36, 63, Math.toRadians(-90));

        drivetrain.setPoseEstimate(initialPose);
        final Trajectory depotPath = drivetrain.trajectoryBuilder(initialPose, Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-36,23,Math.toRadians(-90)), Math.toRadians(-90))
                .splineTo(new Vector2d(-58,12), Math.toRadians(-180))
                .build();


        waitForStart();
        if(isStopRequested()) return;

        drivetrain.followTrajectory(depotPath);

    }
}
