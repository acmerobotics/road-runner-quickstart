package org.firstinspires.ftc.teamcode.Math.AsymmetricProfile;


/**
 * Asymmetric motion profile
 *
 * Traditional motion profiles in FTC simply mirror the acceleration phase to obtain the deceleration
 * this approach is suboptimal because we can generally accelerate faster than we can decelerate
 * So this approach uses a modified version of the roadrunner algorithm to allow for asymmetric constraints
 * This will allow us to accelerate faster while still having good settling performance due to slow deceleration
 */
public class AsymmetricMotionProfile {

    /**
     * the initial position where the robot is
     */
    public final double initialPosition;
    /**
     * the final position we would like to be at
     */
    public final double finalPosition;
    /**
     * the constraints of our robots motion
     */
    public final MotionConstraint constraints;
    /**
     * duration of our acceleration
     */
    protected double dt1;
    /**
     * duration of our coast
     */
    protected double dt2;
    /**
     * duration of our deceleration
     */
    protected double dt3;
    /**
     * the total duration of the motion profile
     *
     */
    protected double profileDuration;

    /**
     * the distance between the initial position and the final position,
     * calculations are easier when we start at 0
     */
    protected double distance;


    /**
     * construct the asymmetric motion profile object
     * @param initialPosition the starting position
     * @param finalPosition the desired end position
     * @param constraints the motion constraints we want to respect between the two positions
     */
    public AsymmetricMotionProfile(double initialPosition, double finalPosition,
                                   MotionConstraint constraints) {
        this.initialPosition = initialPosition;
        this.finalPosition = finalPosition;
        this.constraints = constraints;
        // runs the compute function to find dt1, dt2, and dt3 as well as the profile duration
        compute();
    }

    /**
     * compute dt1, dt2, and dt3 based on the distance and our constraints
     */
    protected void compute() {
        // calculate the distance
        distance = finalPosition - initialPosition;
        /*
        assuming we can accelerate to our maximum speed, the time it will take is equal to the velocity divided by the acceleration
        This is because v = a * t, solving for t gives: t = v/a
         */
        this.dt1 = Math.abs(constraints.max_velocity) / Math.abs(constraints.max_acceleration);
        // same process for dt1 but with acceleration
        this.dt3 = Math.abs(constraints.max_velocity) / Math.abs(constraints.max_deceleration);
        /*
        to calculate dt2 we take the entire distance and divide it by the acceleration from the formula x = v * t, solving for t gives x/v = t
        to solve for dt2 specifically we then need to subtract off the average time on each side of the velocity trapezoid
        This gives us our coast time
         */
        double averageDt = (this.dt1 + this.dt3) / 2;
        this.dt2 = Math.abs(distance) / Math.abs(constraints.max_velocity) - averageDt;


        /*
         * in many cases we will not be going far enough to actually accelerate to our maximum velocity
         * in this case, dt2 will end up being less than zero.
         */
        if (this.dt2 < 0) {
            // we should go ahead and set dt2 to zero so the coast state is just skipped over
            this.dt2 = 0;
            // this if statement assesses which of the two acceleration periods has a higher magnitude
            // and then sets it to the lower to make it symmetrical and respect the minimum constraints of the plant
            if (Math.abs(this.constraints.max_acceleration) > Math.abs(constraints.max_deceleration)) {
                constraints.max_acceleration = Math.abs(Math.abs(constraints.max_deceleration));
            }
            else {
                constraints.max_deceleration = Math.abs(Math.abs(constraints.max_acceleration));
            }
            // after finding out
            this.dt1 = Math.sqrt(Math.abs(distance)/Math.abs(constraints.max_acceleration));
            this.dt3 = Math.sqrt(Math.abs(distance)/Math.abs(constraints.max_deceleration));
        }
        // the length of the motion profile is as simple as summing each period together
        this.profileDuration = this.dt1 + this.dt2 + this.dt3;

    }

    /**
     * calculates where the robot should be based off of a time index
     *
     * @param seconds the current time stamp, recommended to use {@link com.qualcomm.robotcore.util.ElapsedTime} to perform this
     * @return the motion state (desired position, velocity, acceleration) at that given point in time
     *
     * For usage it is recommended to index the motion profile and then take the desired states of
     * the motion state and put them into a feedback / feedforward controller(s)
     *
     */
    public MotionState calculate(double seconds) {
        // the current acceleration
        double acceleration;
        // the current velocity
        double velocity;
        // the current position
        double position;

        // acceleration period
        if (seconds <= this.dt1) {
            // the acceleration in the beginning is just the acceleration
            // we take the absolute value because we are afraid of what the end user might put here
            acceleration = Math.abs(constraints.max_acceleration);
            // simple kinematic equation for velocity (v = a * t)
            velocity = seconds * acceleration;
            /*
            slightly more complex kinematic equation where x = v0 * t + 0.5 * a * t ^ 2
            and the initial velocity (v0) is zero so it simplifies to x = 0.5 * a * t ^ 2
             */
            position = 0.5 * acceleration * Math.pow(seconds, 2);
        } else if (seconds <= this.dt1 + this.dt2) {
            // we are coasting at constant velocity so the acceleration is zero
            acceleration = 0;
            // the velocity should be the same as the velocity at the very end of the acceleration period (at t = dt1)
            velocity = Math.abs(calculate(this.dt1).v);
            // position is equal to the position at the end of dt1 + v * (t - dt1)
            // TODO Assess if we should actually be using constraints.max_velocity or the velocity at the end of dt1
            // Theoretically they should be the same?
            position = Math.abs(calculate(this.dt1).x) + constraints.max_velocity * (seconds - this.dt1);
        } else if (seconds <= this.dt1 + this.dt2 + this.dt3) {
            // once again we do not trust people and we should not trust people...
            acceleration = Math.abs(constraints.max_deceleration);
            // the velocity at the end of the coast period is also the same as the velocity when it began at dt1
            double coastVelocity = Math.abs(calculate(this.dt1).v);
            // the equation v = v0 - (t - dt1-dt2) * a gives us our current velocity
            velocity = coastVelocity - (seconds - this.dt1 - this.dt2) * acceleration;
            // the time at the end of the coast period, when we begin decelerating
            double endofdt2 = this.dt1 + this.dt2;
            // the position at the end of the coast period
            double endOfdt2Pos = Math.abs(calculate(endofdt2).x);
            position = endOfdt2Pos + coastVelocity * (seconds - endofdt2) - 0.5 *
                    acceleration * Math.pow(seconds - endofdt2, 2);
            // uhh, we make it negative lol
            acceleration *= -1;

        } else {
            // if we are out of bounds with time, we just assume we are supposed to be at the end of the profile
            acceleration = 0;
            velocity = 0;
            position = distance;
        }

        // construct the motion state object.  We take the signum of the distance to accurately describe the direction of travel.
        // this is because in the previous calculations our excessive use of Math.abs would only work for positive directions
        return new MotionState(this.initialPosition + position * Math.signum(distance),
                velocity * Math.signum(distance), acceleration * Math.signum(distance));
    }

}