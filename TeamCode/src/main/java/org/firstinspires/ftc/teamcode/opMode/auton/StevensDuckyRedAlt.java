package org.firstinspires.ftc.teamcode.opMode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.CapVision;
import org.firstinspires.ftc.teamcode.hardware.Carousel;
import org.firstinspires.ftc.teamcode.hardware.DelayCommand;
import org.firstinspires.ftc.teamcode.hardware.FreightSensor;
import org.firstinspires.ftc.teamcode.hardware.LiftScoringV2;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class StevensDuckyRedAlt extends LinearOpMode {
    private Acquirer intake = new Acquirer();
    private CapVision cv = new CapVision();
    private Carousel carousel = new Carousel();
    private DelayCommand delay = new DelayCommand();
    private FreightSensor sensor = new FreightSensor();
    private LiftScoringV2 scoringMech = new LiftScoringV2();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    public static double startx = -36.0;
    public static double starty = 70.0;
    public static double startAng = Math.toRadians(90);

    public static double scoreHubPosx = -34;
    public static double scoreHubPosy = 40;

    public static double scoreHubPosAngB = -25;
    public static double scoreHubPosAngR = 15;

    public static double carouselPosx = -62;
    public static double carouselPosy = 64;
    public static double carouselPosAng = Math.toRadians(270);

    public static double parkX = -63;
    public static double parkY = 42;
    public static double parkAng = Math.toRadians(180);

    public static double reposX = -34;
    public static double reposY = -36;

    public static double duckX = -58;
    public static double duckY = -65;

    public static String goal = "midgoal";

    Pose2d startPosR = new Pose2d(startx, -starty, -startAng);
    Vector2d scoreHubPosR = new Vector2d(scoreHubPosx, -scoreHubPosy);
    Pose2d carouselPosR = new Pose2d(carouselPosx, -carouselPosy, carouselPosAng);
    Pose2d parkR = new Pose2d(parkX, -parkY, parkAng);


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        //initialize mechanisms
        carousel.init(hardwareMap);
        scoringMech.init(hardwareMap);
        sensor.init(hardwareMap);
        cv.init(hardwareMap);
        intake.init(hardwareMap);

        //drive train + async updates of mechanisms
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setSlides(scoringMech);

        //important coordinates here

        //set startPose
        drive.setPoseEstimate(startPosR);

        //trajectory
        TrajectorySequence duckyPath = drive.trajectorySequenceBuilder(startPosR)
                .setReversed(true)
                .splineTo(scoreHubPosR, Math.toRadians(scoreHubPosAngR))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringMech.release();
                })
                .waitSeconds(1)
                // slides
                .lineToSplineHeading(carouselPosR)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    carousel.run(false, true);
                })
                .waitSeconds(7)
                .lineToSplineHeading(new Pose2d(reposX, reposY, Math.toRadians(270)))
                .lineTo(new Vector2d( reposX, reposY - 12))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.intake(1);
                })
                .splineTo(new Vector2d(-40, duckY-2), Math.toRadians(180))
                //.splineTo(new Vector2d(duckX, duckY), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(duckX, duckY-2, Math.toRadians(225)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.intake(0);
                })
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringMech.toggle("midgoal");
                })
                .splineTo(scoreHubPosR, Math.toRadians(scoreHubPosAngR))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringMech.release();
                })
                .waitSeconds(1)
                .lineToSplineHeading(parkR)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    carousel.run(false, false);
                })

                .build();

        //3ftx3ftmovement

//        TrajectorySequenceBuilder taahkbeer = drive.trajectorySequenceBuilder(alFatihah.build().end())
//                .splineTo(new Vector2d(bankcurveX,bankcurveY),Math.toRadians(90))
//                .addDisplacementMarker(()->{
//                    drive.acquirerRuns = true;
//                })
//                .forward(tuningNumber)
//                .waitSeconds(2);

//        TrajectorySequenceBuilder allahhuackbar = drive.trajectorySequenceBuilder(taahkbeer.build().end())
//                .back(tuningNumber)
//                .addDisplacementMarker(() -> {
//                    scoringMech.toggle("highgoal");
//                    drive.acquirerRuns = false;
//                })
//                .setReversed(true)
//                .splineTo(new Vector2d(18, starty), Math.toRadians(0))
//                .UNSTABLE_addTemporalMarkerOffset(0,()->{
//                    scoringMech.release();
//                });


        //mashallah

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }
        if (cv.whichRegion() == 1) {
            goal = "highgoal";
        }
        if (cv.whichRegion() == 2) {
            goal = "midgoal";
        }
        if (cv.whichRegion() == 3) {
            goal = "lowgoal";
        }
        telemetry.addData("goal: ", goal);
        telemetry.addData("region", cv.whichRegion());
        telemetry.update();

        scoringMech.toggle(goal);
        drive.followTrajectorySequence(duckyPath);
    }
}
