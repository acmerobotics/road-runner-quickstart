package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.TankDrive;
import com.acmerobotics.roadrunner.followers.GVFFollower;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.profile.SimpleMotionConstraints;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.TankConstraints;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * Base class with shared functionality for sample tank drives. All hardware-specific details are
 * handled in subclasses.
 */
@Config
public abstract class SampleTankDriveBase extends TankDrive {
    // these coefficients are used for point turns only
    public static PIDCoefficients HEADING_PID = new PIDCoefficients();
    // these coefficients are used for all other paths
    public static double kN = 0.0;
    public static double kOmega = 0.0;

    private Pose2d lastError = new Pose2d();

    private DriveConstraints constraints;
    private TankTurnFollower turnFollower;
    private GVFFollower pathFollower;

    public SampleTankDriveBase() {
        super(DriveConstants.TRACK_WIDTH);

        constraints = new TankConstraints(DriveConstants.BASE_CONSTRAINTS, DriveConstants.TRACK_WIDTH);

        turnFollower = new TankTurnFollower(this, HEADING_PID,
                DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic,
                new Pose2d(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Math.toRadians(2)), 0.5);
        pathFollower = new GVFFollower(this,
                new SimpleMotionConstraints(constraints.maximumVelocity, constraints.maximumAcceleration),
                new Pose2d(0.5, 0.5, Math.toRadians(5)),
                kN, kOmega, DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic);
    }

    public PathBuilder pathBuilder() {
        return new PathBuilder(getPoseEstimate());
    }

    public void turn(double angle) {
        turnFollower.followTrajectory(new TrajectoryBuilder(getPoseEstimate(), constraints)
            .turn(angle)
            .build());
    }

    public void turnTo(double heading) {
        turnFollower.followTrajectory(new TrajectoryBuilder(getPoseEstimate(), constraints)
            .turnTo(heading)
            .build());
    }

    public void followPath(Path path) {
        pathFollower.followPath(path);
    }

    public void updateFollower() {
        if (turnFollower.isFollowing()) {
            turnFollower.update(getPoseEstimate());
            lastError = turnFollower.getLastError();
        } else if (pathFollower.isFollowing()) {
            pathFollower.update(getPoseEstimate());
            lastError = pathFollower.getLastError();
        } else {
            setVelocity(new Pose2d());
        }
    }

    public void update() {
        updatePoseEstimate();
        updateFollower();
    }

    public boolean isFollowing() {
        return turnFollower.isFollowing() || pathFollower.isFollowing();
    }

    public Pose2d getFollowingError() {
        return lastError;
    }

    public abstract PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode);

    public abstract void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients);
}
