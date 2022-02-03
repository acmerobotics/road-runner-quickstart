package org.firstinspires.ftc.teamcode.opMode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.Carousel;
import org.firstinspires.ftc.teamcode.hardware.DelayCommand;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.LiftScoringV2;
import org.firstinspires.ftc.teamcode.hardware.MeccRobot;
import org.firstinspires.ftc.teamcode.hardware.ScoringArm;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class LetItRipBlue extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private DelayCommand delay = new DelayCommand();

    public static double tuningNumber = 18;
    @Override
    public void runOpMode() throws InterruptedException {
        LiftScoringV2 scoringMech = new LiftScoringV2();
        scoringMech.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Initialize Mechs
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //Creating trajectories and paths

        double startx = 0;
        double starty = -72;

        double carouselX = -52;
        double parkX = 56;

        Runnable releaseBox = new Runnable() {
            @Override
            public void run() {
                scoringMech.release();
            }
        };

        Pose2d startPos = new Pose2d(startx,starty, Math.toRadians(180));

        TrajectorySequence taahkbeer = drive.trajectorySequenceBuilder(startPos)
                .addDisplacementMarker(()->{
                    scoringMech.toggle("highgoal");

                })
                .back(tuningNumber)
                .addDisplacementMarker(()->{
                    scoringMech.release();
                })

                .build();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }

        drive.followTrajectorySequence(taahkbeer);



    }
}