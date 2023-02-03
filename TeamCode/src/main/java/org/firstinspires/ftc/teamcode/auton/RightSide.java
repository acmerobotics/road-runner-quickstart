package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.oldCode.AutoCommon;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class RightSide extends AutoCommon {

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    @Override
    public void runOpMode() {

        super.runOpMode();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Pose2d startPose = new Pose2d(34.5,-61.5,Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        Trajectory traj = drive.trajectoryBuilder(startPose)
                .back(5)
                .build();





        robot.lift.servoClaw.setPosition(robot.lift.CLAW_CLOSE_POS);
        sleep(500);
        hover();
        sleep(500);
        drive.followTrajectory(traj);
        liftPos(550);
        sleep(500);

        Pose2d addPose = new Pose2d(34.5,-56.5,Math.toRadians(270));

        drive.setPoseEstimate(addPose);

        Trajectory add = drive.trajectoryBuilder(addPose)
                .lineToConstantHeading(new Vector2d(34.5,-6))
                .build();
        drive.followTrajectory(add);
        turretRight45();
        sleep(2000);
        robot.lift.servoExtension.setPosition(robot.lift.EXTENSION_SCORE_45_POS);
        sleep(750);
        highJunctionDrop();
        sleep(500);
        robot.lift.servoClaw.setPosition(robot.lift.CLAW_OPEN_POS);
        sleep(500);
        intake();
        sleep(1000);
        robot.lift.servoClaw.setPosition(robot.lift.CLAW_CLOSE_POS);
        drive.turn(Math.toRadians(-90));
        sleep(100);
        liftPos(450);
        robot.lift.servoExtension.setPosition(robot.lift.EXTENSION_AUTO_POS);
        robot.lift.servoClaw.setPosition(robot.lift.CLAW_OPEN_POS);

        Pose2d secondPose = new Pose2d(34.5,-6.5,Math.toRadians(-180));

        drive.setPoseEstimate(secondPose);

        Trajectory traj1 = drive.trajectoryBuilder(secondPose)
                .lineToConstantHeading(new Vector2d(51,-10),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        drive.followTrajectory(traj1);

        // strafe

        robot.lift.servoClaw.setPosition(robot.lift.CLAW_CLOSE_POS);
        sleep(1000);
        liftPos(760);
        sleep(500);
        robot.lift.servoExtension.setPosition(robot.lift.EXTENSION_INTAKE_POS);
        robot.lift.servoClaw.setPosition(robot.lift.CLAW_CLOSE_POS);
        // pickup

        Pose2d thirdPose = new Pose2d(51,-10,Math.toRadians(-180));

        drive.setPoseEstimate(thirdPose);

        Trajectory traj3 = drive.trajectoryBuilder(thirdPose)
                .lineToConstantHeading(new Vector2d(24.5,-10),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        drive.followTrajectory(traj3);
        //forward

        liftPos(2000);
        sleep(750);
        turretLeft90();
        robot.lift.servoClaw.setPosition(robot.lift.CLAW_CLOSE_POS);
        sleep(500);
        robot.lift.servoExtension.setPosition(robot.lift.EXTENSION_90_AUTO_POS);
        sleep(2000);
        midJunctionDrop();
        sleep(500);
        robot.lift.servoClaw.setPosition(robot.lift.CLAW_OPEN_POS);
        sleep(500);
        robot.lift.servoExtension.setPosition(robot.lift.EXTENSION_INTAKE_POS);
        liftPos(2300);
        turret0Pos();
        sleep(1000);
        intake();
        sleep(1000);
        //drop cone


//        Pose2d fourthPose = new Pose2d(24.5,-10,Math.toRadians(-180));
//
//        drive.setPoseEstimate(fourthPose);
//
//        Trajectory traj4 = drive.trajectoryBuilder(fourthPose)
//                .lineToConstantHeading(new Vector2d(51,-10))
//                .build();
//        drive.followTrajectory(traj4);
//        //comback
//
//        liftPos(320);
//        robot.lift.servoExtension.setPosition(robot.lift.EXTENSION_AUTO_POS);
//        robot.lift.servoClaw.setPosition(robot.lift.CLAW_OPEN_POS);
//        sleep(500);
//        robot.lift.servoClaw.setPosition(robot.lift.CLAW_CLOSE_POS);
//        sleep(500);
//        liftPos(1000);
//        sleep(500);
//        robot.lift.servoExtension.setPosition(robot.lift.EXTENSION_INTAKE_POS);
        //pickup
//
//
//        Pose2d fifthPose = new Pose2d(51,-10,Math.toRadians(-180));
//
//        drive.setPoseEstimate(fifthPose);
//
//        Trajectory traj5 = drive.trajectoryBuilder(fifthPose)
//                .lineToConstantHeading(new Vector2d(24.5,-10))
//                .build();
//        drive.followTrajectory(traj5);
//        //forward
//
//        turretLeft90();
//        sleep(500);
//        robot.lift.servoExtension.setPosition(robot.lift.EXTENSION_90_AUTO_POS);
//        sleep(1000);
//        midJunctionDrop();
//        sleep(500);
//        robot.lift.servoClaw.setPosition(robot.lift.CLAW_OPEN_POS);
//        sleep(500);
//        robot.lift.servoExtension.setPosition(robot.lift.EXTENSION_INTAKE_POS);
//        robot.lift.servoClaw.setPosition(robot.lift.CLAW_INIT_POS);
//        intake();
//        sleep(1000);

        //dropoff

        if (tagOfInterest == null) {
            Pose2d sixthPose = new Pose2d(24.5,-10,Math.toRadians(-180));

            drive.setPoseEstimate(sixthPose);

            Trajectory traj6 = drive.trajectoryBuilder(sixthPose)
                    .lineToConstantHeading(new Vector2d(64,-10))
                    .build();
            drive.followTrajectory(traj6);
            //trajetory (code that makes it move)
        } else if (tagOfInterest.id == LEFT) {
            Pose2d sixthPose = new Pose2d(24.5,-10,Math.toRadians(-180));

            drive.setPoseEstimate(sixthPose);

            Trajectory traj6 = drive.trajectoryBuilder(sixthPose)
                    .lineTo(new Vector2d(10.5,-10))
                    .build();
            drive.followTrajectory(traj6);

            Pose2d seventhPose = new Pose2d(34.5,-10,Math.toRadians(-180));

            drive.setPoseEstimate(seventhPose);

            Trajectory traj7 = drive.trajectoryBuilder(seventhPose)
                    .lineToConstantHeading(new Vector2d(34.5,-25))
                    .build();
            drive.followTrajectory(traj7);
            sleep(100000);
        } else if (tagOfInterest.id == MIDDLE) {
            Pose2d sixthPose = new Pose2d(24.5,-10,Math.toRadians(-180));

            drive.setPoseEstimate(sixthPose);

            Trajectory traj6 = drive.trajectoryBuilder(sixthPose)
                    .lineToConstantHeading(new Vector2d(36.5,-10))
                    .build();
            drive.followTrajectory(traj6);

            Pose2d seventhPose = new Pose2d(36.5,-10,Math.toRadians(-180));

            drive.setPoseEstimate(seventhPose);

            Trajectory traj7 = drive.trajectoryBuilder(seventhPose)
                    .lineToConstantHeading(new Vector2d(36.5,-25))
                    .build();
            drive.followTrajectory(traj7);
            sleep(100000);

        } else if (tagOfInterest.id == RIGHT) {
            Pose2d sixthPose = new Pose2d(24.5,-10,Math.toRadians(-180));

            drive.setPoseEstimate(sixthPose);

            Trajectory traj6 = drive.trajectoryBuilder(sixthPose)
                    .lineToConstantHeading(new Vector2d(64,-10))
                    .build();
            drive.followTrajectory(traj6);
            //trajetory (code that makes it move)
        }

    }
}
