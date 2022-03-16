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
import org.firstinspires.ftc.teamcode.hardware.RetractableOdoSys;
import org.firstinspires.ftc.teamcode.hardware.util.DelayCommand;
import org.firstinspires.ftc.teamcode.hardware.FreightSensor;
import org.firstinspires.ftc.teamcode.hardware.LiftScoringV2;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous (group = "RedAuton")
public class StevensDuckyRedW extends LinearOpMode {
    private Acquirer intake = new Acquirer();
    private CapVision cv = new CapVision();
    private Carousel carousel = new Carousel();
    private DelayCommand delay = new DelayCommand();
    private FreightSensor sensor = new FreightSensor();
    private LiftScoringV2 scoringMech= new LiftScoringV2();
    private RetractableOdoSys odoSys = new RetractableOdoSys();

    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    public static double startx = -36.0;
    public static double starty = 70.0;
    public static double startAng = Math.toRadians(90);

    public static double scoreHubPosx = -34;
    public static double scoreHubPosy = 38;

    public static double scoreHubPosAngB = -25;
    public static double scoreHubPosAngR = 30;

    public static double carouselPosx = -62;
    public static double carouselPosy = 62;
    public static double carouselPosAng = Math.toRadians(180);

    public static double parkX = -60;
    public static double parkY = 40;
    public static double parkAng = Math.toRadians(180);

    public static double reposX = -34;
    public static double reposY = 36;

    public static double preSweepY = 48;
    public static double sweepX = -34;
    public static double sweepY = 67;

    public static double duckX = -58;
    public static double preParkY = 48;

    public static double enterX = 15;
    public static double enterY = 71.5;
    public static String goal = "highgoal";

    public static double parkTimer = 1750;
    public static double parkAngleOffset = 7;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        //initialize mechanisms
        intake.init(hardwareMap);
        carousel.init(hardwareMap);
        scoringMech.init(hardwareMap);
        sensor.init(hardwareMap);
        cv.init(hardwareMap);
        odoSys.init(hardwareMap, true);

        //drive train + async updates of mechanisms
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setSlides(scoringMech);

        //important coordinates here
        Pose2d startPosR = new Pose2d(startx, -starty, -startAng);
        Vector2d scoreHubPosR = new Vector2d(scoreHubPosx, -scoreHubPosy);
        Pose2d carouselPosR = new Pose2d(carouselPosx, -carouselPosy, carouselPosAng);
        Pose2d parkR = new Pose2d(parkX, -parkY, parkAng);
        Pose2d reposition = new Pose2d(reposX, -reposY, Math.toRadians(270));
        Vector2d preSweep = new Vector2d(reposX,-preSweepY);
        Vector2d sweepPos = new Vector2d(sweepX, -sweepY);
        Pose2d postSweep = new Pose2d(duckX, -sweepY, Math.toRadians(210));
        Pose2d prePark = new Pose2d(scoreHubPosx, -preParkY, Math.toRadians(0));
        Pose2d bEnter = new Pose2d(enterX, -enterY, Math.toRadians(0));

        //set startPose
        drive.setPoseEstimate(startPosR);

        //trajectory
        TrajectorySequence duckyPath = drive.trajectorySequenceBuilder(startPosR)
                .waitSeconds(1)
                .setReversed(true)
                .splineTo(scoreHubPosR,Math.toRadians(scoreHubPosAngR))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    scoringMech.releaseSoft();
                })
                .waitSeconds(1)
                //slides
                .lineToSplineHeading(carouselPosR)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    carousel.run(false,true);
                })
                .waitSeconds(3.0)
                //carousel
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    carousel.run(false,false);
                })
                .lineToSplineHeading(reposition)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.intake(1);
                })
                .lineTo(preSweep)
                .splineTo(sweepPos, Math.toRadians(180))
                //.splineTo(new Vector2d(duckX, duckY), Math.toRadians(180))
                .lineToLinearHeading(postSweep)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.intake(0);
                })
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringMech.toggle("highgoal");
                })
                .splineTo(scoreHubPosR, Math.toRadians(scoreHubPosAngR))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringMech.releaseSoft();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    carousel.run(false, false);
                })
                .lineToLinearHeading(prePark)
                .forward(15)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    odoSys.toggle();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    delay.delay(()->{
                        this.requestOpModeStop();
                    }, (int)parkTimer);
                })
                .forward(45)
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
        if(cv.whichRegion() == 1) {
            goal = "lowgoal";
        }
        if(cv.whichRegion() == 2) {
            goal = "midgoal";
        }
        if(cv.whichRegion() == 3) {
            goal = "highgoal";
        }
        telemetry.addData("goal: ",goal);
        telemetry.addData("region", cv.whichRegion());
        telemetry.update();

        scoringMech.toggle(goal);
        drive.followTrajectorySequence(duckyPath);

    }
}