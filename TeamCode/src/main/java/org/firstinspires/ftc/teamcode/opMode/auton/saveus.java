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
public class saveus extends LinearOpMode {
    private CapVision cv = new CapVision();
    private Carousel carousel = new Carousel();
    private DelayCommand delay = new DelayCommand();
    private FreightSensor sensor = new FreightSensor();
    private LiftScoringV2 scoringMech = new LiftScoringV2();
    private RetractableOdoSys odoSys = new RetractableOdoSys();
    private Acquirer intake = new Acquirer();

    public static double startx = 15.0;
    public static double starty = 70.0;
    public static double startAng = Math.toRadians(90);

    public static double scoreHubPosx = 6;
    public static double scoreHubPosy = 52;

    public static double scoreHubPosAngB = 60;
    public static double scoreHubPosAngR = -40;

    public static double repositionX = 15.0;
    public static double reposistionY = 71.5;

    public static double preSplineY = 53.5;
    public static double bEnterX = 20;
    public static double bExitX = 30;
    public static double bEnterY = 68;
    public static double warehouseX = 53;
    public static double bExitY = -70.5;
    public static double inc = 0;
    public static Pose2d startPos = new Pose2d(startx, starty, startAng);

    public static Pose2d firstRepos = new Pose2d(scoreHubPosx + 8 , scoreHubPosy + 8, Math.toRadians(22.5));
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static String goal = "highgoal";

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        //initialize mechanisms
//        carousel.init(hardwareMap);

        sensor.init(hardwareMap);
        odoSys.init(hardwareMap, true);
//
        scoringMech.init(hardwareMap, sensor);
//        cv.init(hardwareMap);
//        odoSys.init(hardwareMap, true);
        intake.init(hardwareMap);
        Vector2d scoreHubPosB = new Vector2d(scoreHubPosx, scoreHubPosy);
        Vector2d preSpline = new Vector2d(scoreHubPosx, preSplineY);
        Vector2d bEnter = new Vector2d(bEnterX, bEnterY);


        //drive train + async updates of mechanisms
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setSlides(scoringMech);
        drive.setAcquirer(intake, sensor);

        //important coordinates here
        //set startPose

        //trajectory


        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }
//        if (cv.whichRegion() == 1) {
//            goal = "lowgoal";
//        }
//        if (cv.whichRegion() == 2) {
//            goal = "midgoal";
//        }
//        if (cv.whichRegion() == 3) {
//            goal = "highgoal";
//        }
//        telemetry.addData("goal: ", goal);
//        telemetry.addData("region", cv.whichRegion());
//        telemetry.update();
//
//        scoringMech.toggle(goal);
        drive.setPoseEstimate(startPos);
        TrajectorySequence depoPath = drive.trajectorySequenceBuilder(startPos)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(scoreHubPosB, Math.toRadians(scoreHubPosAngB)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //scoringMech.releaseHard();
                    // drive.acquirerRuns = true;
                })
                .waitSeconds(.1)
                .lineTo(preSpline)
                .splineToSplineHeading(new Pose2d(bEnter, Math.toRadians(0)), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(warehouseX-1, bEnterY))                        .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //scoringMech.toggle("highgoal");
                    // drive.acquirerRuns = false;
                })
                .lineTo(new Vector2d(bExitX, bEnterY))
                .splineTo(new Vector2d(scoreHubPosx, scoreHubPosy), Math.toRadians(scoreHubPosAngB+180))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // scoringMech.releaseHard();
                    // drive.acquirerRuns = true;
                })
                .waitSeconds(.1)
                .lineTo(preSpline)
                .splineToSplineHeading(new Pose2d(bEnter, Math.toRadians(0)), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(warehouseX-1, bEnterY))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //scoringMech.toggle("highgoal");
                    // drive.acquirerRuns = false;
                })
                .lineTo(new Vector2d(bExitX, bEnterY))
                .splineTo(new Vector2d(scoreHubPosx, scoreHubPosy), Math.toRadians(scoreHubPosAngB+180))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // scoringMech.releaseHard();
                    // drive.acquirerRuns = true;
                })
                .lineTo(preSpline)
                .splineToSplineHeading(new Pose2d(bEnter, Math.toRadians(0)), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(warehouseX-1, bEnterY))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //scoringMech.toggle("highgoal");
                    // drive.acquirerRuns = false;
                })
                .lineTo(new Vector2d(bExitX, bEnterY))
                .splineTo(new Vector2d(scoreHubPosx, scoreHubPosy), Math.toRadians(scoreHubPosAngB+180))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // scoringMech.releaseHard();
                    // drive.acquirerRuns = true;
                })
                .lineTo(preSpline)
                .splineToSplineHeading(new Pose2d(bEnter, Math.toRadians(0)), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(warehouseX-1, bEnterY))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //scoringMech.toggle("highgoal");
                    // drive.acquirerRuns = false;
                })
                .lineTo(new Vector2d(bExitX, bEnterY))
                .splineTo(new Vector2d(scoreHubPosx, scoreHubPosy), Math.toRadians(scoreHubPosAngB+180))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // scoringMech.releaseHard();
                    // drive.acquirerRuns = true;
                })
                .lineTo(preSpline)
                .splineToSplineHeading(new Pose2d(bEnter, Math.toRadians(0)), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(warehouseX-1, bEnterY))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //scoringMech.toggle("highgoal");
                    // drive.acquirerRuns = false;
                })
                .lineTo(new Vector2d(bExitX, bEnterY))
                .splineTo(new Vector2d(scoreHubPosx, scoreHubPosy), Math.toRadians(scoreHubPosAngB+180))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // scoringMech.releaseHard();
                    // drive.acquirerRuns = true;
                })
                .lineTo(preSpline)
                .splineToSplineHeading(new Pose2d(bEnter, Math.toRadians(0)), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(warehouseX-10, bEnterY))
                .build();
        drive.followTrajectorySequence(depoPath);
    }

    public static Vector2d pose2Vector(Pose2d givenPose){
        return new Vector2d(givenPose.getX(),givenPose.getY());
    }
}
