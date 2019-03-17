package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.Kinematics;
import com.acmerobotics.roadrunner.drive.TankDrive;
import com.acmerobotics.roadrunner.drive.TankKinematics;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.jetbrains.annotations.NotNull;

import java.util.List;

public class TankTurnFollower extends TrajectoryFollower {
    private TankDrive drive;
    private PIDFController headingController;
    private double kV, kA, kStatic;
    private Pose2d lastError;

    public TankTurnFollower(TankDrive drive, PIDCoefficients headingCoeffs,
                            double kV, double kA, double kStatic,
                            Pose2d admissibleError, double timeout) {
        super(admissibleError, timeout);

        this.drive = drive;
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
        lastError = new Pose2d();
        headingController = new PIDFController(headingCoeffs);
    }

    @NotNull
    @Override
    public Pose2d getLastError() {
        return lastError;
    }

    @Override
    protected void setLastError(@NotNull Pose2d newError) {
        lastError = newError;
    }

    @Override
    public void update(@NotNull Pose2d currentPose) {
        if (!isFollowing()) {
            drive.setVelocity(new Pose2d());
            return;
        }

        double t = elapsedTime();

        Trajectory traj = getTrajectory();
        Pose2d targetPose = traj.get(t);
        Pose2d targetPoseVel = traj.velocity(t);
        Pose2d targetPoseAccel = traj.acceleration(t);

        if (Math.abs(targetPoseVel.getX()) > 1e-2 || Math.abs(targetPoseVel.getY()) > 1e-2) {
            throw new RuntimeException(getClass().getSimpleName() +
                    " only supports trajectories with angular velocity");
        }

        Pose2d targetRobotPoseVel = Kinematics.fieldToRobotPoseVelocity(targetPose, targetPoseVel);
        Pose2d targetRobotPoseAccel = Kinematics.fieldToRobotPoseAcceleration(targetPose, targetPoseVel, targetPoseAccel);

        Pose2d poseError = Kinematics.calculatePoseError(targetPose, currentPose);

        headingController.setTargetPosition(poseError.getHeading());

        double headingCorrection = headingController.update(0.0, targetRobotPoseVel.getHeading());

        Pose2d correctedVelocity = targetRobotPoseVel.plus(new Pose2d(0.0, 0.0, headingCorrection));

        List<Double> wheelVelocities = TankKinematics.robotToWheelVelocities(correctedVelocity, drive.getTrackWidth());
        List<Double> wheelAccelerations = TankKinematics.robotToWheelAccelerations(targetRobotPoseAccel, drive.getTrackWidth());

        List<Double> motorPowers = Kinematics.calculateMotorFeedforward(wheelVelocities, wheelAccelerations, kV, kA, kStatic);

        drive.setMotorPowers(motorPowers.get(0), motorPowers.get(1));

        lastError = poseError;
    }
}
