package org.firstinspires.ftc.teamcode.opMode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.CapVision;
import org.firstinspires.ftc.teamcode.hardware.Carousel;
import org.firstinspires.ftc.teamcode.hardware.FreightSensor;
import org.firstinspires.ftc.teamcode.hardware.LiftScoringV2;
import org.firstinspires.ftc.teamcode.hardware.RetractableOdoSys;
import org.firstinspires.ftc.teamcode.hardware.util.DelayCommand;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous (group = "BlueAuton")
public class PetersCyclesBlueIshVersion extends LinearOpMode {
    private CapVision cv = new CapVision();
    private Carousel carousel = new Carousel();
    private DelayCommand delay = new DelayCommand();
    private FreightSensor sensor = new FreightSensor();
    private LiftScoringV2 scoringMech = new LiftScoringV2();
    private RetractableOdoSys odoSys = new RetractableOdoSys();

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double startx = 15.0;
    public static double starty = 70.0;
    public static double startAng = Math.toRadians(90);

    public static double scoreHubPosx = 7;
    public static double scoreHubPosy = 43;

    public static double scoreHubPosAngB = 40;
    public static double repositionX = 15.0;
    public static double reposistionY = 71.5;

    public static double distanceForwards = 30;
    public static double strafeDistance = 24;


    public static String goal = "midgoal";

    Pose2d startPosB = new Pose2d(startx, starty, startAng);
    Vector2d scoreHubPosB = new Vector2d(scoreHubPosx, scoreHubPosy);
    Pose2d repositionB = new Pose2d(repositionX, reposistionY, Math.toRadians(0));


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        //initialize mechanisms
        carousel.init(hardwareMap);
        scoringMech.init(hardwareMap);
        sensor.init(hardwareMap);
        cv.init(hardwareMap);
        odoSys.init(hardwareMap, true);


        //drive train + async updates of mechanisms
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setSlides(scoringMech);

        //important coordinates here
        Pose2d startPos = new Pose2d(startx, starty, startAng);
        Vector2d scoreHubPos = new Vector2d(scoreHubPosx, scoreHubPosy);

        //set startPose
        drive.setPoseEstimate(startPos);

        //trajectory
        TrajectorySequence depoPath = drive.trajectorySequenceBuilder(startPos)
                .waitSeconds(2)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(scoreHubPosB, Math.toRadians(scoreHubPosAngB)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // scoringMech.release();
                })
                .waitSeconds(1)
                // .lineToLinearHeading(repositionB)
                .splineTo(new Vector2d(repositionX, reposistionY - 4), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(repositionX + distanceForwards, reposistionY), Math.toRadians(
                        0))
                //.lineToLinearHeading(new Pose2d(repositionX + distanceForwards, reposistionY,Math.toRadians(0)))
                .UNSTABLE_addDisplacementMarkerOffset(0, ()->{
                    // intake here
                })
                .setReversed(true)
                .waitSeconds(1)

                .splineTo(new Vector2d(repositionX + 5, reposistionY), Math.toRadians(180))

                .splineTo(new Vector2d(scoreHubPosx, scoreHubPosy),
                        Math.toRadians(220))
                .UNSTABLE_addDisplacementMarkerOffset(0, ()->{
                    //score here
                })
                .waitSeconds(1)
                .setReversed(false)
                .splineTo(new Vector2d(repositionX, reposistionY - 4), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(repositionX + distanceForwards, reposistionY), Math.toRadians(
                        0))
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
        drive.followTrajectorySequence(depoPath);
    }
}
