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
public class stevenMode extends LinearOpMode {
    private Acquirer acquirer = new Acquirer();
    private Carousel carousel = new Carousel();
    private DelayCommand delay = new DelayCommand();
    private FreightSensor sensor = new FreightSensor();
    private LiftScoringV2 scoringMech= new LiftScoringV2();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double tuningNumber = 40;
    public static double tuningTimer = 1;


    public static double startx = 0;
    public static double starty = -72;

    public static double bankcurveX = -3;
    public static double bankcurveY = starty + 22;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        acquirer.init(hardwareMap);
        carousel.init(hardwareMap);
        scoringMech.init(hardwareMap);
        sensor.init(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setSlides(scoringMech);
        drive.setAcquirer(acquirer,sensor);


        Pose2d startPos = new Pose2d(startx,starty, Math.toRadians(180));
        drive.setPoseEstimate(startPos);

        TrajectorySequenceBuilder alFatihah = drive.trajectorySequenceBuilder(startPos)
                .setReversed(false)
                .addDisplacementMarker(()->{
                    scoringMech.toggle("highgoal");
                })
                .UNSTABLE_addTemporalMarkerOffset(tuningTimer,()->{
                    scoringMech.release();
                })
                .back(18)
                .waitSeconds(3);

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

        int cycles = 1;
        for(int i = 0; i < cycles; i++){
            alFatihah = alFatihah
                    //start of taahkbeer
                    .splineTo(new Vector2d(bankcurveX,bankcurveY),Math.toRadians(90))
                    .addDisplacementMarker(()->{
                        drive.acquirerRuns = true;
                    })
                    .forward(tuningNumber)
                    .waitSeconds(2)
                    //start of allahhuackbar
                    .back(tuningNumber)
                    .addDisplacementMarker(() -> {
                        scoringMech.toggle("highgoal");
                        drive.acquirerRuns = false;
                    })
                    .setReversed(true)
                    .splineTo(new Vector2d(18, starty), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(0,()->{
                        scoringMech.release();
                    });
        }

        //mashallah
        alFatihah = alFatihah
                    .splineTo(new Vector2d(bankcurveX,bankcurveY),Math.toRadians(90))
                    .forward(tuningNumber);

        TrajectorySequence mashallah = alFatihah.build();
//        TrajectorySequenceBuilder taahkbeers = drive.trajectorySequenceBuilder(startPos)
//                .setReversed(false)
//                .addDisplacementMarker(()->{
//                    scoringMech.toggle("highgoal");
//                })
//                .UNSTABLE_addTemporalMarkerOffset(tuningTimer,()->{
//                    scoringMech.release();
//                })
//                .back(18)
//                .waitSeconds(3)
//                .splineTo(new Vector2d(bankcurveX,bankcurveY),Math.toRadians(90))
//                .addDisplacementMarker(()->{
//                    drive.acquirerRuns = true;
//                })
//                .forward(tuningNumber)
//                .waitSeconds(2)
//                .back(tuningNumber)
//                .addDisplacementMarker(() -> {
//                    scoringMech.toggle("highgoal");
//                    drive.acquirerRuns = false;
//                })
//                .setReversed(true)
//                .splineTo(new Vector2d(18, starty), Math.toRadians(0))
//                .UNSTABLE_addTemporalMarkerOffset(0,()->{
//                    scoringMech.release();
//                })
//                .waitSeconds(3);


//                .addDisplacementMarker(() -> scoringMech.release())
//                .waitSeconds(3)

//        TrajectorySequence taahkbeer = drive.trajectorySequenceBuilder(startPos)
//                .addDisplacementMarker(()->{
//                    scoringMech.toggle("highgoal");
//
//                })
//                .back(tuningNumber)
//                .addDisplacementMarker(()->{
//                    scoringMech.release();
//                })
//
//                .build();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }

       drive.followTrajectorySequence(mashallah);
    }
}
