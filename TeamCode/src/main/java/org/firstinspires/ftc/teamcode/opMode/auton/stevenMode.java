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

@Config
@Autonomous
public class stevenMode extends LinearOpMode {
    private Acquirer acquirer = new Acquirer();
    private Carousel carousel = new Carousel();
    private DelayCommand delay = new DelayCommand();
    private FreightSensor sensor = new FreightSensor();
    private LiftScoringV2 scoringMech= new LiftScoringV2();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double tuningNumber = 18;
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

        Runnable toggleSlides = new Runnable() {
            @Override
            public void run() {
                scoringMech.toggle("highgoal");
            }
        };

        Pose2d startPos = new Pose2d(startx,starty, Math.toRadians(180));
        drive.setPoseEstimate(startPos);
        TrajectorySequence allahuackbar = drive.trajectorySequenceBuilder(startPos)
                .addDisplacementMarker(()->{
                    scoringMech.toggle("highgoal");

                })
                .back(tuningNumber)
                .addTemporalMarker(tuningTimer,()->{
                    scoringMech.release();
                })
                .waitSeconds(3)
                .splineTo(new Vector2d(bankcurveX,bankcurveY),Math.toRadians(90))
                .addDisplacementMarker(()->{
                    drive.acquirerRuns = true;
                })
                .forward(48)
                .build();

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

        drive.followTrajectorySequence(allahuackbar);
        while(!isStopRequested()){
            telemetry.addData("Status","doing other stuff");
            telemetry.update();
        }
    }
}
