package org.firstinspires.ftc.teamcode.opMode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.CapVision;
import org.firstinspires.ftc.teamcode.hardware.Carousel;
import org.firstinspires.ftc.teamcode.hardware.RetractableOdoSys;
import org.firstinspires.ftc.teamcode.hardware.util.DelayCommand;
import org.firstinspires.ftc.teamcode.hardware.FreightSensor;
import org.firstinspires.ftc.teamcode.hardware.LiftScoringV2;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous (group = "BlueAuton")
public class StevensDuckyBlue extends LinearOpMode {
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

    public static double scoreHubPosx = -32;
    public static double scoreHubPosy = 25;

    public static double scoreHubPosAngB = -45
            ;
    public static double scoreHubPosAngR = 25;

    public static double carouselPosx = -62;
    public static double carouselPosy = 62;
    public static double carouselPosAng = Math.toRadians(180);

    public static double parkX = -60;
    public static double parkY = 44;
    public static double parkAng = Math.toRadians(180);

    public static String goal = "highgoal";

    Pose2d startPosB = new Pose2d(startx, starty, startAng);
    Vector2d scoreHubPosB = new Vector2d(scoreHubPosx, scoreHubPosy);
    Pose2d carouselPosB = new Pose2d(carouselPosx, carouselPosy, carouselPosAng);
    Pose2d parkB = new Pose2d(parkX, parkY, parkAng);


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        //initialize mechanisms
        carousel.init(hardwareMap);
        sensor.init(hardwareMap);
        scoringMech.init(hardwareMap, sensor);
        cv.init(hardwareMap);
        odoSys.init(hardwareMap, true);


        //drive train + async updates of mechanisms
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setSlides(scoringMech);

        //important coordinates here
        Pose2d startPos = new Pose2d(startx,starty, startAng);
        Vector2d scoreHubPos = new Vector2d(scoreHubPosx,scoreHubPosy);
        Pose2d carouselPos = new Pose2d(carouselPosx,carouselPosy,carouselPosAng);
        Pose2d park = new Pose2d(parkX,parkY,parkAng);

        //set startPose
        drive.setPoseEstimate(startPos);

        //trajectory
        TrajectorySequence duckyPath = drive.trajectorySequenceBuilder(startPos)
                .waitSeconds(1)
                .setReversed(true)
                .splineTo(new Vector2d(parkX, parkY),Math.toRadians(90))
                .splineTo(scoreHubPosB, Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    scoringMech.releaseHard();
                })
                .waitSeconds(1)
                //slides
                .setReversed(false)
                .splineTo(new Vector2d(parkX, parkY), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    carousel.run(true,false);
                })
                .build();
        TrajectorySequence parka = drive.trajectorySequenceBuilder(carouselPosB)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    carousel.run(false,false);
                })
                .lineToSplineHeading(parkB)
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
        waitForStart();
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
        ElapsedTime timer = new ElapsedTime();
        while(timer.seconds() <= 2) {
            if(timer.seconds() <= 1) {
                carousel.rrrun(timer, 1);
            }else {
                carousel.runmax(true, false);
            }
        }

        drive.followTrajectorySequence(parka);
    }
}