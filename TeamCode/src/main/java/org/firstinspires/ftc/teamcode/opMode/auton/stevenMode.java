package org.firstinspires.ftc.teamcode.opMode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.Carousel;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.ScoringArm;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous
public class stevenMode extends LinearOpMode {
    private Acquirer acquirer = new Acquirer();
    private Carousel carousel = new Carousel();
    private ScoringArm scoringArm = new ScoringArm();
    public Lift lift = new Lift();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        acquirer.init(hardwareMap);
        carousel.init(hardwareMap);
        scoringArm.init(hardwareMap);
        lift.init(hardwareMap);
        lift.setTargetPosition(0.0);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-47, -62, Math.toRadians(180));
        drive.setPoseEstimate(startPose);
        Trajectory traj1 = drive.trajectoryBuilder(startPose, true)
                .forward(10)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(startPose, true)
                .splineTo(new Vector2d(-12, -36), Math.toRadians(90))
                .build();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }
        drive.followTrajectory(traj1);

        drive.followTrajectory(traj2);
    }
}
