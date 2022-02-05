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
        Pose2d startPos = new Pose2d(startx,starty,0);
        drive.setPoseEstimate(startPos);
        TrajectorySequence alFatihah = drive.trajectorySequenceBuilder(startPos)
                .setReversed(false)
                .addDisplacementMarker(()->{
                    scoringMech.toggle("highgoal");
                })
                .UNSTABLE_addTemporalMarkerOffset(1,()->{
                    scoringMech.release();
                })
                .back(18)
                .waitSeconds(0.1)
                .lineToSplineHeading(new Pose2d(-3,0,270))
                .forward(40)
                .build();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }

        drive.followTrajectorySequence(alFatihah);




    }
}
