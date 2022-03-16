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
public class TestingCycles extends LinearOpMode {
    private CapVision cv = new CapVision();
    private Carousel carousel = new Carousel();
    private DelayCommand delay = new DelayCommand();
    private FreightSensor sensor = new FreightSensor();
    private LiftScoringV2 scoringMech = new LiftScoringV2();
    private RetractableOdoSys odoSys = new RetractableOdoSys();
    private Acquirer intake = new Acquirer();

    public static double scoreHubPosx = 0;
    public static double scoreHubPosy = 45;

    public static double wareHousePosX = 48;
    public static double warhousePosY = 64;

    public static Pose2d origin = new Pose2d(15,70,Math.toRadians(90));
    public static Pose2d startPos = new Pose2d(scoreHubPosx,scoreHubPosy,Math.toRadians(45));
    public static Pose2d endPos = new Pose2d(wareHousePosX,warhousePosY, Math.toRadians(0));
    public static Pose2d midWayPos = new Pose2d(20,64,Math.toRadians(0));
    public static Vector2d endPosVector = new Vector2d(wareHousePosX,warhousePosY);
    public static Vector2d startPosVector = new Vector2d(scoreHubPosx,scoreHubPosy);

    public static Pose2d firstRepos = new Pose2d(scoreHubPosx + 8 , scoreHubPosy + 8, Math.toRadians(22.5));
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static String goal = "highgoal";

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        //initialize mechanisms
//        carousel.init(hardwareMap);
        scoringMech.init(hardwareMap);
        sensor.init(hardwareMap);
//        cv.init(hardwareMap);
//        odoSys.init(hardwareMap, true);
        intake.init(hardwareMap);


        //drive train + async updates of mechanisms
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setSlides(scoringMech);
        drive.setAcquirer(intake, sensor);

        //important coordinates here
        //set startPose

        //trajectory
        TrajectorySequence depoPath = drive.trajectorySequenceBuilder(origin)
                .addDisplacementMarker(()->{
                    scoringMech.toggle(goal);
                })
                .setReversed(true)
                .splineTo(pose2Vector(firstRepos), Math.toRadians(180) + firstRepos.getHeading())
                .splineTo(pose2Vector(startPos), startPos.getHeading() + Math.toRadians(180))
                .UNSTABLE_addDisplacementMarkerOffset(0,()->{
                    scoringMech.releaseHard();
                    drive.acquirerRuns = true;
                })
                //END OF SCORE PRELOAD
                // .setReversed(true)
                // .splineTo(pose2Vector(startPos), startPos.getHeading())
                .setReversed(false)
                .splineTo(pose2Vector(midWayPos), midWayPos.getHeading())
                .splineTo(pose2Vector(endPos), endPos.getHeading())
                .setReversed(true)

                .splineTo(pose2Vector(midWayPos), midWayPos.getHeading() + Math.toRadians(180))
                .splineTo(pose2Vector(startPos),  startPos.getHeading() + Math.toRadians(180))
                .setReversed(false)

                //cycle1
//                .splineTo(pose2Vector(midWayPos), midWayPos.getHeading())
//                .splineTo(pose2Vector(endPos), endPos.getHeading())
//                .setReversed(true)
//
//                .splineTo(pose2Vector(midWayPos), midWayPos.getHeading() + Math.toRadians(180))
//                .splineTo(pose2Vector(startPos), startPos.getHeading() + Math.toRadians(180))
//                .setReversed(false)
//
//                //cycle 2
//                .splineTo(pose2Vector(midWayPos), midWayPos.getHeading())
//                .splineTo(pose2Vector(endPos), endPos.getHeading())
//                .setReversed(true)
//
//                .splineTo(pose2Vector(midWayPos), midWayPos.getHeading() + Math.toRadians(180))
//                .splineTo(pose2Vector(startPos), startPos.getHeading() + Math.toRadians(180))
//                .setReversed(false)
//
//                //cycle 3
//                .splineTo(pose2Vector(midWayPos), midWayPos.getHeading())
//                .splineTo(pose2Vector(endPos), endPos.getHeading())
//                .setReversed(true)
//
//                .splineTo(pose2Vector(midWayPos), midWayPos.getHeading() + Math.toRadians(180))
//                .splineTo(pose2Vector(startPos), startPos.getHeading() + Math.toRadians(180))
//                .setReversed(false)
//
//                //cycle 4
//                .splineTo(pose2Vector(midWayPos), midWayPos.getHeading())
//                .splineTo(pose2Vector(endPos), endPos.getHeading())
//                .setReversed(true)
//
//
//                .splineTo(pose2Vector(midWayPos), midWayPos.getHeading() + Math.toRadians(180))
//                .splineTo(pose2Vector(startPos), startPos.getHeading() + Math.toRadians(180))
//                .setReversed(false)
//
//                //cycle 5
//                .splineTo(pose2Vector(midWayPos), midWayPos.getHeading())
//                .splineTo(pose2Vector(endPos), endPos.getHeading())
//                .setReversed(true)
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
        drive.setPoseEstimate(origin);
        drive.followTrajectorySequence(depoPath);
    }

    public static Vector2d pose2Vector(Pose2d givenPose){
        return new Vector2d(givenPose.getX(),givenPose.getY());
    }
}
