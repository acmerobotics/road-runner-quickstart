package org.firstinspires.ftc.teamcode.auton;

import static org.firstinspires.ftc.teamcode.oldCode.RobotHardware.lift;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.oldCode.AutoCommon;
import org.firstinspires.ftc.teamcode.oldCode.RobotHardware;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class RightCYCLEAuto extends AutoCommon {

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    public static int turret = 0;

    @Override
    public void runOpMode() {

        super.runOpMode();

        Lift autoLift = new Lift(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Pose2d startPose = new Pose2d(34.5,-61.5,Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        Trajectory traj = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(34.5,-32),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(3, () -> {
                    lift.motorLiftR.setTargetPosition(lift.LIFT_MID_POS);
                })
                .build();



        robot.lift.servoClaw.setPosition(robot.lift.CLAW_CLOSE_POS);
        sleep(250);
        drive.followTrajectory(traj);
        turretRight45();
        robot.lift.servoExtension.setPosition(robot.lift.EXTENSION_SCORE_45_POS);
        sleep(1500);
        midJunctionDrop();
        // score preload cone

        lift.motorLiftR.setTargetPosition(lift.LIFT_MID_POS);
        lift.Turret0();
        sleep(2500);
        Pose2d secondPose = new Pose2d(34.5,-32,Math.toRadians(270));

        drive.setPoseEstimate(secondPose);

        Trajectory traj2 = drive.trajectoryBuilder(secondPose)
                .lineToConstantHeading(new Vector2d(34.5,-37),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(0.1, () -> {
                    lift.motorLiftR.setTargetPosition(lift.LIFT_INTAKE_POS);
                })
                .build();
        drive.followTrajectory(traj2);
        sleep(2000);
        // setup for signal cone

        Pose2d thirdPose = new Pose2d(34.5,-37,Math.toRadians(270));

        drive.setPoseEstimate(thirdPose);

        Trajectory traj3 = drive.trajectoryBuilder(thirdPose)
                .lineToConstantHeading(new Vector2d(34.5,-6),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(1, () -> {
                    lift.servoClaw.setPosition(lift.CLAW_OPEN_POS);
                })
                .addDisplacementMarker(6, () -> {
                    lift.servoClaw.setPosition(lift.CLAW_CLOSE_POS);
                })
                .addDisplacementMarker(8, () -> {
                    lift.motorLiftR.setTargetPosition(lift.LIFT_MID_POS);
                })

                .build();
        drive.followTrajectory(traj3);
        //pickup signal cone

        drive.turn(Math.toRadians(-90));
        dropSignal();
        // drop signal cone

        Pose2d fourthPose = new Pose2d(34.5,-6,Math.toRadians(270));

        drive.setPoseEstimate(fourthPose);

        Trajectory traj4 = drive.trajectoryBuilder(fourthPose)
                .lineToConstantHeading(new Vector2d(34.5,8))
                .addDisplacementMarker(2, () -> {
                    lift.servoExtension.setPosition(lift.EXTENSION_AUTO_POS);
                })
                .build();
        drive.followTrajectory(traj4);
        stackPickupCone();
        // pickup cone #5 in stack

        Pose2d fifthPose = new Pose2d(34.5,8,Math.toRadians(270));

        drive.setPoseEstimate(fifthPose);

        Trajectory traj5 = drive.trajectoryBuilder(fifthPose)
                .lineToConstantHeading(new Vector2d(34.5,-6))
                .addDisplacementMarker(2, () -> {
                    lift.motorLiftR.setTargetPosition(lift.LIFT_MID_POS);
                })
                .build();
        drive.followTrajectory(traj5);
        dropLeft135Cone();









        sleep(50000);


        drive.update();
        // We update our lift PID continuously in the background, regardless of state
        autoLift.updateLift();
        // turret update

        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();


        // Print pose to telemetry
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();
    }
}




// Assume we have a hardware class called lift
// Lift uses a PID controller to maintain its height
// Thus, update() must be called in a loop
    class Lift {
    public Lift(HardwareMap hardwareMap) {
        RobotHardware robot = null;

        robot.lift.motorLiftR.setTargetPosition(0);
        robot.lift.motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorLiftR.setPower(1);
    }

    public void updateLift() {
    }

}

