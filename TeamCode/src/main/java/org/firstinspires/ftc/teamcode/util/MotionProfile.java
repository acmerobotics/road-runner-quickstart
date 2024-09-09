package org.firstinspires.ftc.teamcode.util;

public class MotionProfile {

    double maxvel;
    double maxaccel;

    double acceleration_dt;
    double distance;
    double halfway_distance;
    double acceleration_distance;
    double deceleration_dt;
    double cruise_distance;
    double cruise_dt;
    double deceleration_time;

    public MotionProfile(double start, double end, double maxvel, double maxaccel) {
        // Calculate the time it takes to accelerate to max velocity
        acceleration_dt = maxvel / maxaccel;
        distance = end - start;

        // If its going backwards, make sure to make accel negative
        if (distance < 0) {
            maxaccel = -maxaccel;
        }

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        halfway_distance = distance / 2;
        acceleration_distance = 0.5 * maxaccel * Math.pow(acceleration_dt, 2);

        if (acceleration_distance > halfway_distance) {
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * maxaccel));
        }

        acceleration_distance = 0.5 * maxaccel * Math.pow(acceleration_dt, 2);

        // recalculate max velocity based on the time we have to accelerate and decelerate
        maxvel = maxaccel * acceleration_dt;

        // we decelerate at the same rate as we accelerate
        deceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        cruise_distance = distance - (2 * acceleration_distance);
        cruise_dt = cruise_distance / maxvel;
        deceleration_time = acceleration_dt + cruise_dt;

        this.maxvel = maxvel;
        this.maxaccel = maxaccel;
    }

    public double get(double time) {
        double cruise_current_dt;

        // check if we're still in the motion profile
        double entire_dt = acceleration_dt + cruise_dt + deceleration_dt;
        if (time > entire_dt) {
            return distance;
        }

        // if we're accelerating
        if (time < acceleration_dt) {
            // use the kinematic equation for acceleration
            return 0.5 * maxaccel * Math.pow(time, 2);
        }

        // if we're cruising
        else if (time < deceleration_time) {
            acceleration_distance = 0.5 * maxaccel * Math.pow(acceleration_dt, 2);
            cruise_current_dt = time - acceleration_dt;

            // use the kinematic equation for constant velocity (i luv mrs moore)
            return acceleration_distance + (maxvel * cruise_current_dt);
        }

        // if we're decelerating
        else {
            acceleration_distance = 0.5 * maxaccel * Math.pow(acceleration_dt, 2);
            cruise_distance = maxvel * cruise_dt;
            deceleration_time = time - deceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            return acceleration_distance + cruise_distance + (maxvel * deceleration_time - 0.5 * maxaccel * Math.pow(deceleration_time, 2));
        }
    }
}
