package org.firstinspires.ftc.teamcode.drive.locolization;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;


import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.util.math.Pose;

import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

@Config
public class TwoWheelLocalizer extends TwoTrackingWheelLocalizer implements SwerveLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.689;
    public static double GEAR_RATIO = 1;

    public static double PARALLEL_X = 0;
    public static double PARALLEL_Y = 1.7374; // 1.16442

    public static double PERPENDICULAR_X = -4.4149; // -3.61212 old
    public static double PERPENDICULAR_Y = 0;

    private final DoubleSupplier horizontalPosition, lateralPosition, imuAngle;

    public TwoWheelLocalizer(BrainSTEMRobot robot) {

        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.horizontalPosition = () -> robot.parallelPod.getPosition();
        this.lateralPosition = () -> robot.perpindicularPod.getPosition();
        this.imuAngle = robot::getAngle;

    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return imuAngle.getAsDouble();
    }

    @Override
    public Double getHeadingVelocity() {
        return 0.0;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(horizontalPosition.getAsDouble()),
                encoderTicksToInches(lateralPosition.getAsDouble())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(0.0, 0.0);
    }

    @Override
    public Pose getPos() {
        Pose2d pose = getPoseEstimate();
        return new Pose(pose.getX(), -pose.getY(), pose.getHeading());
    }

    @Override
    public void setPos(Pose pose) {
    }

    @Override
    public void periodic() {
        super.update();
    }
}