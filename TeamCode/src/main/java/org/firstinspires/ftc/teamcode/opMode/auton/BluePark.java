package org.firstinspires.ftc.teamcode.opMode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.LiftScoringV2;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class BluePark extends LinearOpMode {
    double startx = 0;
    double starty = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        LiftScoringV2 scoringMech = new LiftScoringV2();
        scoringMech.init(hardwareMap);

        drive.setSlides(scoringMech);

        Pose2d startPos = new Pose2d(startx,starty,Math.toRadians(0));
        drive.setPoseEstimate(startPos);
        TrajectorySequence alFatihah = drive.trajectorySequenceBuilder(startPos)
                .setReversed(false)
                .addDisplacementMarker(()->{
                    scoringMech.toggle("highgoal");
                })
                .UNSTABLE_addTemporalMarkerOffset(2,()->{
                    scoringMech.release();
                })
                .lineToLinearHeading(new Pose2d(-18,24,Math.toRadians(0)))
//                .back(18)
//                .waitSeconds(0.1)
//                .lineToLinearHeading(new Pose2d(5,0,Math.toRadians(270)))
//                .lineToSplineHeading(new Pose2d(3,52,Math.toRadians(270)))
                .waitSeconds(5)
                .build();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }
        sleep(13000);
        drive.followTrajectorySequence(alFatihah);




    }
}
