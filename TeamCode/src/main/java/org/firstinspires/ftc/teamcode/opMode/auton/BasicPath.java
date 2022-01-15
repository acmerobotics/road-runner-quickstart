package org.firstinspires.ftc.teamcode.opMode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.Carousel;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.ScoringArm;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class BasicPath extends LinearOpMode {
    private Acquirer acquirer = new Acquirer();
    private Carousel carousel = new Carousel();
    private ScoringArm scoringArm = new ScoringArm();
    public Lift lift = new Lift();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize Mechs
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        acquirer.init(hardwareMap);
        carousel.init(hardwareMap);
        scoringArm.init(hardwareMap);
        lift.init(hardwareMap);
        lift.setTargetPosition(0.0);

        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Creating trajectories and paths

        double startx = -16;
        double starty = -70;

        double carouselX = -52;
        double parkX = 56;

        Pose2d startPos = new Pose2d(startx,starty, Math.toRadians(180));

        TrajectorySequence duckPark = drive.trajectorySequenceBuilder(startPos)
                .forward(startx - carouselX)
                .addTemporalMarker(1,()->{
                    carousel.run(true);
                })
                .waitSeconds(3)
                .forward(carouselX - parkX)
                .addTemporalMarker(0,()->{
                    carousel.run(false);
                })
                .build();
        TrajectorySequence park = drive.trajectorySequenceBuilder(startPos)
                .forward(startx - parkX)
                .build();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }

        if (isStopRequested()) return;
        //Auton Route begins here

        drive.followTrajectorySequence(park);


    }
}
