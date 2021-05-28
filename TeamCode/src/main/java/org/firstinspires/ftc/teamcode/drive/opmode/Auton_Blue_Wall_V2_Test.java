package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.Auton_Base;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

/*
 * This is a straight line autonomous that will drop 1 wobble goal at square A on both sides of the field
 */

@Autonomous(group = "drive")
public class Auton_Blue_Wall_V2_Test extends Auton_Base {

    @Override
    public void runOpMode() throws InterruptedException {
        ringsDetected = initHardware_Vision();

        // Starting Position
        Pose2d startPose = new Pose2d(-63, 48, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        //Init trajectories

        //Shoot position
        traj0 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-10, 46, Math.toRadians(0)))
                .build();

        //Initialize paths based on rings detected
        switch (ringsDetected) {
        //Case C:
            case "Quad":
            trajC1 = drive.trajectoryBuilder(traj0.end())
                    .lineTo(new Vector2d(56, 48))
                    .build();

            trajC2 = drive.trajectoryBuilder(trajC1.end(), true)
                    .splineToConstantHeading(new Vector2d(-18, 21), Math.toRadians(180))
                    .build();

            trajC3 = drive.trajectoryBuilder(trajC2.end())
                    .lineTo(new Vector2d(-28, 19),
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();

            trajC4 = drive.trajectoryBuilder(trajC3.end())
                    .strafeTo(new Vector2d(56, 19))
                    .build();

            trajC5 = drive.trajectoryBuilder(trajC4.end())
                    .lineToLinearHeading(new Pose2d(56, 37, Math.toRadians(-90)))
                    .build();

            trajC6 = drive.trajectoryBuilder(trajC5.end(), false)
                    .strafeTo(new Vector2d(56, 30))
                    .build();

            break;

        //Case B:
            case "Single":
            trajB1 = drive.trajectoryBuilder(traj0.end())
    //                .lineTo(new Vector2d(-24, 48))
                    .lineTo(new Vector2d(36, 19))
                    .build();

            trajB2 = drive.trajectoryBuilder(trajB1.end())
                    .strafeTo(new Vector2d(-25, 19))
                    .build();

            trajB3 = drive.trajectoryBuilder(trajB2.end())
                    .lineTo(
                            new Vector2d(-27, 19),
                            SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();

            trajB4 = drive.trajectoryBuilder(trajB3.end())
                    .lineToSplineHeading(new Pose2d(20, 19, Math.toRadians(-135)))
                    .build();

            trajB5 = drive.trajectoryBuilder(trajB4.end(), false)
                    .forward(6)
                    .build();
            break;

        //Case A:
        case "None":
            trajA1 = drive.trajectoryBuilder(traj0.end())
                    .lineToSplineHeading(new Pose2d(12, 48, Math.toRadians(0)))
                    .build();

            trajA2 = drive.trajectoryBuilder(trajA1.end())
                    .lineToConstantHeading(new Vector2d(12, 19))
                    .build();

            trajA3 = drive.trajectoryBuilder(trajA2.end())
                    .lineToConstantHeading(new Vector2d(-25, 19))
                    .build();

            trajA4 = drive.trajectoryBuilder(trajA3.end())
                    .lineTo(new Vector2d(-29, 19),
                            SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();

            trajA5 = drive.trajectoryBuilder(trajA4.end())
                    .lineToConstantHeading(new Vector2d(10, 19))
                    .build();

            trajA6 = drive.trajectoryBuilder(trajA5.end())
                    .lineToLinearHeading(new Pose2d(10, 36, Math.toRadians(-90)))
                    .build();

            trajA7 = drive.trajectoryBuilder(trajA6.end(), false)
                    .forward(6)
                    .build();

            break;

}
        waitForStart();
        if (isStopRequested()) return;
        //Use Tensorflo to figure out which path to use
        //Test all the paths.
        telemetry.clear();

        switch(ringsDetected) {
            case "Quad": pathC(); break;
            case "Single": pathB(); break;
            case "None": pathA(); break;
            default: telemetry.addData("Default", "Default");
        }
        telemetry.update();


//        sleep(5000);

//        if (tfod != null)
//            tfod.shutdown();


//        wobbleDropper.setPosition(1);
//        drive.followTrajectory(
//                drive.trajectoryBuilder(traj.end(), true)
//                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
//                        .build()
//        );

    }

    private void pathA() {
        pathShoot();
        drive.followTrajectory(trajA1);
        drive.wobbleDrop();
        drive.moveTo("Down");
        sleep(1000);
        drive.followTrajectory(trajA2);
        drive.followTrajectory(trajA3);
        drive.followTrajectory(trajA4);
        drive.wobbleGrab();
        sleep(1000);
        drive.moveTo("Carry");
        drive.followTrajectory(trajA5);
        drive.followTrajectory(trajA6);
        drive.moveTo("Down");
        drive.wobbleRelease();
        sleep(1000);
        drive.followTrajectory(trajA7);
//        sleep(2000);
//        drive.moveTo("Away");
    }

    private void pathB() {
        pathShoot();
        drive.followTrajectory(trajB1);
        drive.wobbleDrop();
        drive.moveTo("Down");
        sleep(1000);
        drive.followTrajectory(trajB2);
        drive.followTrajectory(trajB3);
        drive.wobbleGrab();
        sleep(2000);
        drive.moveTo("Carry");
        drive.followTrajectory(trajB4);
        drive.moveTo("Down");
        drive.wobbleRelease();
        sleep(2000);
        drive.followTrajectory(trajB5);
    }

    private void pathC() {
        pathShoot();
        drive.followTrajectory(trajC1);
        drive.wobbleDrop();
        drive.moveTo("Down");
        sleep(1000);
        drive.followTrajectory(trajC2);
        drive.followTrajectory(trajC3);
        drive.wobbleGrab();
        sleep(1000);
        drive.moveTo("Carry");
        drive.followTrajectory(trajC4);
        drive.followTrajectory(trajC5);
        drive.moveTo("Down");
        drive.wobbleRelease();
        sleep(1000);
        drive.followTrajectory(trajC6);
    }

}
