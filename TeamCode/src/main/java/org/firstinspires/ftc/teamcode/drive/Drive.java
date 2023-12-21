package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import java.util.List;

import kotlin.Unit;

public interface Drive {
    Pose2d getPoseEstimate();
    void updatePoseEstimate();
    void setPoseEstimate(Pose2d value);

    TrajectoryBuilder trajectoryBuilder(Pose2d startPose);
    TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed);
    TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading);

    TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose);

    void  turnAsync(double angle);
    void turn(double angle);

    void followTrajectoryAsync(Trajectory trajectory);
    void followTrajectory(Trajectory trajectory);
    void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence);
    void followTrajectorySequence(TrajectorySequence trajectorySequence);
    Pose2d getLastError();

    void update();

    void waitForIdle();

    boolean isBusy();

    void setMode(DcMotor.RunMode runMode);

    void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior);
    void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients);
    void setWeightedDrivePower(Pose2d drivePower);

    List<Double> getWheelPositions();
    List<Double> getWheelVelocities();
    Pose2d getPoseVelocity();

    void setDrivePower(Pose2d drivePower );
    void setMotorPowers(double v, double v1);
    void setMotorPowers(double v, double v1, double v2, double v3);
    double getRawExternalHeading();
    Double getExternalHeadingVelocity();

    static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVelocity, double trackWidth) {
        return null;
    }
    static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return null;
    }


}

