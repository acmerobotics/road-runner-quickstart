package org.firstinspires.ftc.teamcode.Math.AsymmetricProfile;


/**
 * the current state of the robot at a given point in time as calculated
 * by {@link AsymmetricMotionProfile}
 */
public class MotionState {
    /**
     * current position
     */
    protected double x;
    /**
     * current velocity
     */
    protected double v;
    /**
     * current acceleration
     */
    protected double a;

    /**
     * Construct a motion state object
     * @param x position
     * @param v velocity
     * @param a acceleration
     */
    public MotionState(double x, double v, double a) {
        this.x = x;
        this.v = v;
        this.a = a;
    }

    /**
     * get the current acceleration
     * @return scalar for acceleration
     */
    public double getA() {
        return a;
    }
    /**
     * get the current velocity
     * @return scalar for velocity
     */
    public double getV() {
        return v;
    }

    /**
     * get the current position
     * @return scalar for position
     */
    public double getX() {
        return x;
    }
}
