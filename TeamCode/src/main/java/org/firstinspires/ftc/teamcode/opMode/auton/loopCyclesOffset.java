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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Config
@Autonomous (group = "BlueAuton")
public class loopCyclesOffset extends LinearOpMode {
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

    public static double scoreHubPosAngB = 60;
    public static double scoreHubPosAngR = -40;

    public static double repositionX = scoreHubPosx;
    public static double reposistionY = 71.5;

    public static double preSplineY = 55.0;
    public static double preSplineX = 2;
    public static double bEnterX = 20;
    public static double bEnterY = 73;
    public static double warehouseX = 43;
    public static double bExitY = 71;
    public static double inc = 0;
    public static Pose2d startPos = new Pose2d(startx, starty, startAng);
    public static double xmod = 5;
    public static double ymod = 5;
    public static double localeReadjustX = 0;
    public static double wallYEstimate = 72.38;
    public static double localeReadjustY = 0;

    public static double depoIncrement = 0;
    public static double scoreHubXOffset = 0;
    public static double intakeAngle = -20;
    public static String goal = "highgoal";
    public static int cycles = 4;
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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setSlides(scoringMech);
        drive.setAcquirer(intake, sensor);

        //important coordinates here
        Pose2d startPos = new Pose2d(startx, starty, startAng);
        Pose2d scoreHubPos = new Pose2d(scoreHubPosx, scoreHubPosy, Math.toRadians(scoreHubPosAngB));
        //set startPose
        drive.setPoseEstimate(startPos);
        TrajectorySequence startPath = drive.trajectorySequenceBuilder(startPos)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(scoreHubPosx, scoreHubPosy, Math.toRadians(scoreHubPosAngB)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringMech.releaseHard();
                    drive.acquirerRuns = true;
                })
                .waitSeconds(.1)
                .build();
        while(!opModeIsActive() && !isStopRequested()) {
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

        scoringMech.toggle(goal);
        drive.followTrajectorySequence(startPath);
        for(int i = 0; i < cycles; i++) {
            TrajectorySequence cycles = drive.trajectorySequenceBuilder(scoreHubPos)
                    .lineTo(new Vector2d(preSplineX, preSplineY))
                    .splineToSplineHeading(new Pose2d(bEnterX, bEnterY, Math.toRadians(0)), Math.toRadians(0))
                    .lineToLinearHeading(new Pose2d(warehouseX, bEnterY))
                    .UNSTABLE_addTemporalMarkerOffset(1.25, () -> {
                        drive.acquirerReverse = true;
                        scoringMech.toggle("highgoal");
                        // drive.acquirerRuns = false;              //This section is "spline into the barrier, then into warehouse, then stop intaking after .1 seconds
                    })
                    //-----------------------------------------------------------------------------------
                    .lineTo(new Vector2d(bEnterX, bExitY))
                    .splineTo(new Vector2d(scoreHubPosx + scoreHubXOffset, scoreHubPosy), Math.toRadians(scoreHubPosAngB+180))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        scoringMech.releaseHard();
                        drive.acquirerReverse = false;   //This section is "line out of the barrier, then spline to the score hub"
                    })
                    .build();


        }

    }
}
