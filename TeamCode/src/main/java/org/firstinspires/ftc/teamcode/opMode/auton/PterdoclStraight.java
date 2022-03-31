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
import org.firstinspires.ftc.teamcode.hardware.FreightSensor;
import org.firstinspires.ftc.teamcode.hardware.LiftScoringV2;
import org.firstinspires.ftc.teamcode.hardware.RetractableOdoSys;
import org.firstinspires.ftc.teamcode.hardware.util.DelayCommand;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous (group = "BlueAuton")
public class PterdoclStraight extends LinearOpMode {
    private CapVision cv = new CapVision();
    private Carousel carousel = new Carousel();
    private DelayCommand delay = new DelayCommand();
    private FreightSensor sensor = new FreightSensor();
    private LiftScoringV2 scoringMech = new LiftScoringV2();
    private RetractableOdoSys odoSys = new RetractableOdoSys();
    private Acquirer intake = new Acquirer();

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double startx = 15.0;
    public static double starty = 70.0;
    public static double startAng = Math.toRadians(90);

    public static double scoreHubPosx = 6;
    public static double scoreHubPosy = 46;
    public static double scoreHubXOffset = -8.0;
    public static double scoreHubYOffset = 5;
    public static double scoreHubAngleOffset = 90;

    public static double scoreHubPosAngB = 60;
    public static double scoreHubPosAngR = -40;

    public static double repositionX = scoreHubPosx;
    public static double reposistionY = 71.5;

    public static double preSplineY = 60;
    public static double preSplineX = 2;

    public static double preSplineVersY = 60;
    public static double preSplineVersX = 5;

    public static double bEnterX = 20;
    public static double bEnterY = 73;
    public static double warehouseX = 43;
    public static double bExitY = 73;
    public static double inc = 0;
    public static Pose2d startPos = new Pose2d(startx, starty, startAng);
    public static double xmod = 7;
    public static double ymod = 5;
    public static double localeReadjustX = 0;
    public static double wallYEstimate = 70 + 2.38;
    public static double localeReadjustY = 0;

    public static double depoIncrement = 1;
    public static double intakeAngle = -20;
    public static String goal = "highgoal";

    Pose2d startPosB = new Pose2d(startx, starty, startAng);
    Pose2d startPosR = new Pose2d(startx, -starty, -startAng);

    Vector2d scoreHubPosB = new Vector2d(scoreHubPosx, scoreHubPosy);
    Vector2d scoreHubPosR = new Vector2d(scoreHubPosx, -scoreHubPosy);

    Pose2d repositionB = new Pose2d(repositionX, reposistionY, Math.toRadians(0));
    Vector2d preSpline = new Vector2d(preSplineX, preSplineY);
    Vector2d preSplineVers = new Vector2d(preSplineVersX, preSplineVersY);
    public static double preSplineVersAng = 30;
    Vector2d bEnter = new Vector2d(bEnterX, bEnterY);
    Vector2d bEnter2 = new Vector2d(bEnterX, bEnterY-inc);
    Vector2d bEnter3 = new Vector2d(bEnterX, bEnterY-2*inc);
    Vector2d bEnter4 = new Vector2d(bEnterX, bEnterY-3*inc);
    Vector2d bEnter5 = new Vector2d(bEnterX, bEnterY-4*inc);
    Vector2d bExit = new Vector2d(bEnterX, bExitY);
    Vector2d bExit2 = new Vector2d(bEnterX, bEnterY-2*inc);
    Vector2d bExit3 = new Vector2d(bEnterX, bEnterY-4*inc);
    Vector2d bExit4 = new Vector2d(bEnterX, bEnterY-6*inc);
    Vector2d wareHouse = new Vector2d(warehouseX, bEnterY);
    Vector2d wareHouse2 = new Vector2d(warehouseX, bEnterY-inc);
    Vector2d wareHouse3 = new Vector2d(warehouseX, bEnterY-2*inc);
    Vector2d wareHouse4 = new Vector2d(warehouseX, bEnterY-3*inc);
    Vector2d wareHouse5 = new Vector2d(warehouseX, bEnterY-4*inc);

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        //initialize mechanisms
        carousel.init(hardwareMap);
        sensor.init(hardwareMap);
        cv.init(hardwareMap);
        odoSys.init(hardwareMap, true);
        intake.init(hardwareMap);

        scoringMech.init(hardwareMap, sensor);


        //drive train + async updates of mechanisms
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setSlides(scoringMech);
        drive.setAcquirer(intake, sensor);

        //important coordinates here
        Pose2d startPos = new Pose2d(startx, starty, startAng);
        //set startPose
        drive.setPoseEstimate(startPos);

        //trajectory
        TrajectorySequence depoPath = drive.trajectorySequenceBuilder(startPos)
                //-----------------------------------------------------------------------------------BEGINNING OF DUMPING PRELOAD
                .lineToLinearHeading(new Pose2d(bEnter, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(warehouseX, bEnterY))
                .lineToLinearHeading(new Pose2d(bEnter, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(scoreHubPosB, Math.toRadians(scoreHubPosAngB)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // scoringMech.releaseHard();
                    // drive.acquirerRuns = true; //This section is "go to score hub from startpos,
                    // score and start intake"
                })

                .lineToLinearHeading(new Pose2d(bEnter, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(warehouseX, bEnterY))
                .lineToLinearHeading(new Pose2d(bEnter, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(scoreHubPosB, Math.toRadians(scoreHubPosAngB)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // scoringMech.releaseHard();
                    // drive.acquirerRuns = true; //This section is "go to score hub from startpos,
                    // score and start intake"
                })

                .lineToLinearHeading(new Pose2d(bEnter, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(warehouseX, bEnterY))
                .lineToLinearHeading(new Pose2d(bEnter, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(scoreHubPosB, Math.toRadians(scoreHubPosAngB)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // scoringMech.releaseHard();
                    // drive.acquirerRuns = true; //This section is "go to score hub from startpos,
                    // score and start intake"
                })

                .lineToLinearHeading(new Pose2d(bEnter, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(warehouseX, bEnterY))
                .lineToLinearHeading(new Pose2d(bEnter, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(scoreHubPosB, Math.toRadians(scoreHubPosAngB)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // scoringMech.releaseHard();
                    // drive.acquirerRuns = true; //This section is "go to score hub from startpos,
                    // score and start intake"
                })

                .lineToLinearHeading(new Pose2d(bEnter, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(warehouseX, bEnterY))
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
            goal = "lowgoal";
        }
        if (cv.whichRegion() == 2) {
            goal = "midgoal";
        }
        if (cv.whichRegion() == 3) {
            goal = "highgoal";
        }
        telemetry.addData("goal: ", goal);
        telemetry.addData("region", cv.whichRegion());
        telemetry.update();

        //scoringMech.toggle(goal);
        drive.followTrajectorySequence(depoPath);
    }

    public void readjustLocale(SampleMecanumDrive drive){
        Pose2d driveCurrent = drive.getPoseEstimate();
        Pose2d poseReadjustment = new Pose2d(
                driveCurrent.getX() + localeReadjustX, driveCurrent.getY(), driveCurrent.getHeading()
        );
        drive.setPoseEstimate(poseReadjustment);
    }
}