package org.firstinspires.ftc.teamcode.Math.AsymmetricProfile;


/**
 * This is a constraint on our motion profile, it describes the dynamics we want our robot to
 * exert over a given trajectory
 */
public class MotionConstraint {

    /**
     * the maximum initial acceleration we allow the robot to experience
     */
    public double max_acceleration;
    /**
     * the maximum final deceleration we allow the robot to experience
     *
     * Generally this is lower than {@link #max_acceleration}
     */
    public double max_deceleration;

    /**
     * the maximum velocity we can reasonably reach in this given trajectory
     */
    public double max_velocity;


    public MotionConstraint(double max_acceleration, double max_deceleration, double max_velocity) {
        this.max_acceleration = max_acceleration;
        this.max_deceleration = max_deceleration;
        this.max_velocity = max_velocity;
    }


}
