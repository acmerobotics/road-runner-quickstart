package org.firstinspires.ftc.teamcode.opMode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.hardware.Carousel;
import org.firstinspires.ftc.teamcode.hardware.DelayCommand;
import org.firstinspires.ftc.teamcode.hardware.FreightSensor;
import org.firstinspires.ftc.teamcode.hardware.LiftScoringV2;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
@Autonomous
public class stevensPark extends LinearOpMode {
    private Acquirer acquirer = new Acquirer();
    private Carousel carousel = new Carousel();
    private DelayCommand delay = new DelayCommand();
    private FreightSensor sensor = new FreightSensor();
    private LiftScoringV2 scoringMech= new LiftScoringV2();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double tuningNumber = 40;
    public static double tuningTimer = 1;


    public static double startx = 0;
    public static double starty = 72;

    public static double bankcurveX = -3.5;
    public static double bankcurveY = starty - 22;
    public static int cycles = 3;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        acquirer.init(hardwareMap);
        carousel.init(hardwareMap);
        scoringMech.init(hardwareMap);
        sensor.init(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setSlides(scoringMech);
        drive.setAcquirer(acquirer, sensor);


        Pose2d startPos = new Pose2d(startx, starty, Math.toRadians(180));
        drive.setPoseEstimate(startPos);
        TrajectorySequenceBuilder steven = drive.trajectorySequenceBuilder(startPos)
                .setReversed(false)
                .addDisplacementMarker(() -> {
                    scoringMech.toggle("highgoal");
                })
                .UNSTABLE_addTemporalMarkerOffset(tuningTimer, () -> {
                    scoringMech.release();
                })
                .back(18)
                .waitSeconds(0.1)
                .back(9)
                .turn(Math.toRadians(90))
                .strafeRight(9.5)
                .forward(tuningNumber + 5);
        TrajectorySequence mashallah = steven.build();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }

        drive.followTrajectorySequence(mashallah);
    }
}
