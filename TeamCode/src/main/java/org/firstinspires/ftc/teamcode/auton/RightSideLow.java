package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.oldCode.AutoCommon;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class RightSideLow extends AutoCommon {
    @Override
    public void runOpMode() {

        super.runOpMode();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Pose2d startPose = new Pose2d(34.5,-61.5,Math.toRadians(270));

        drive.setPoseEstimate(startPose);





        Trajectory traj = drive.trajectoryBuilder(startPose)
                .back(54)
                .build();





        robot.lift.servoClaw.setPosition(robot.lift.CLAW_CLOSE_POS);
        sleep(500);
        hover();
        sleep(500);
        drive.followTrajectory(traj);
        turretRight45();
        sleep(2000);
        robot.lift.servoExtension.setPosition(robot.lift.EXTENSION_SCORE_45_POS);
        sleep(1200);
        highJunctionDrop();
        sleep(500);
        robot.lift.servoClaw.setPosition(robot.lift.CLAW_OPEN_POS);
        sleep(500);
        intake();
        sleep(1000);
        robot.lift.servoClaw.setPosition(robot.lift.CLAW_CLOSE_POS);
        drive.turn(Math.toRadians(-90));
        sleep(100);
        liftPos(500);
        robot.lift.servoExtension.setPosition(robot.lift.EXTENSION_AUTO_POS);
        robot.lift.servoClaw.setPosition(robot.lift.CLAW_OPEN_POS);

        Pose2d secondPose = new Pose2d(34.5,-7.5,Math.toRadians(-180));

        drive.setPoseEstimate(secondPose);

        Trajectory traj1 = drive.trajectoryBuilder(secondPose)
                .lineToConstantHeading(new Vector2d(51,-9),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        drive.followTrajectory(traj1);

        robot.lift.servoClaw.setPosition(robot.lift.CLAW_CLOSE_POS);
        sleep(750);
        liftPos(1000);
        sleep(500);
        robot.lift.servoExtension.setPosition(robot.lift.EXTENSION_INTAKE_POS);
//         pickup

        turretLeftLow90();
        robot.lift.servoExtension.setPosition(robot.lift.EXTENSION_90_AUTO_LOW_POS);
        sleep(2000);
        robot.lift.servoClaw.setPosition(robot.lift.CLAW_OPEN_POS);
        sleep(500);
        robot.lift.servoExtension.setPosition(robot.lift.EXTENSION_INTAKE_POS);
        turret0Pos();
        sleep(750);
        liftPos(320);
        //first cycle done













    }
}
