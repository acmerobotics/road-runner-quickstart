package org.firstinspires.ftc.teamcode.opMode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.Carousel;
import org.firstinspires.ftc.teamcode.hardware.DelayCommand;
import org.firstinspires.ftc.teamcode.hardware.FreightSensor;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.LiftScoringV2;
import org.firstinspires.ftc.teamcode.hardware.ScoringArm;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;

@Config
@Autonomous
public class StevensDuckyBlue extends LinearOpMode {
    private Carousel carousel = new Carousel();
    private DelayCommand delay = new DelayCommand();
    private FreightSensor sensor = new FreightSensor();
    private LiftScoringV2 scoringMech= new LiftScoringV2();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    public static double startx = -35.0;
    public static double starty = 70.0;
    public static double startAng = Math.toRadians(90);

    public static double scoreHubPosx = -34;
    public static double scoreHubPosy = 40;
    public static double scoreHubPosAng = -25;

    public static double carouselPosx = -62;
    public static double carouselPosy = 62;
    public static double carouselPosAng = Math.toRadians(180);

    public static double parkX = -60;
    public static double parkY = 40;
    public static double parkAng = Math.toRadians(180);

    public static String goal = "highgoal";

    Pose2d startPos = new Pose2d(startx,starty, startAng);
    Vector2d scoreHubPos = new Vector2d(scoreHubPosx,scoreHubPosy);
    Pose2d carouselPos = new Pose2d(carouselPosx,carouselPosy,carouselPosAng);
    Pose2d park = new Pose2d(parkX,parkY,parkAng);


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        //initialize mechanisms
        carousel.init(hardwareMap);
        scoringMech.init(hardwareMap);
        sensor.init(hardwareMap);

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
                .setReversed(true)
                .splineTo(scoreHubPos,Math.toRadians(scoreHubPosAng))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    scoringMech.release();
                })
                .waitSeconds(1)
                //slides
                .lineToSplineHeading(carouselPos)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    carousel.run(true,false);
                })
                .waitSeconds(7)
                //carousel
                .lineToSplineHeading(park)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    carousel.run(false,false);
                })
                .build();

        TrajectorySequence duckyPathHighGoal = drive.trajectorySequenceBuilder(startPos)
                .setReversed(true)
                .splineTo(scoreHubPos,Math.toRadians(-25))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    scoringMech.release();
                })
                .waitSeconds(1)
                //slides
                .lineToSplineHeading(carouselPos)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    carousel.run(true,false);
                })
                .waitSeconds(7)
                //carousel
                .lineToSplineHeading(park)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    carousel.run(false,false);
                })
                .build();
               // .splineTo(new Vector2d());

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
        scoringMech.toggle(goal);
        drive.followTrajectorySequence(duckyPath);
    }
}