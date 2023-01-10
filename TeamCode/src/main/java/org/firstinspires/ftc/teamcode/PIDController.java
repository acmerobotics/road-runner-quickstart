package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;


public class PIDController {

    double Kp;
    double Ki;
    double Kd;
    double lastError = 0;
    double integral = 0;
    boolean angleWrap = false;

    ElapsedTime timer = new ElapsedTime();

    /**
     * Set PID gains
     * @param Kp proportional gain
     * @param Ki integral gain
     * @param Kd derivative gain
     */
    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public PIDController(double Kp, double Ki, double Kd, boolean angleWrap) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.angleWrap = angleWrap;
    }

    /**
     * calculate PID output given the reference and the current system state
     * @param reference where we would like our system to be
     * @param state where our system is
     * @return the signal to send to our motor or other actuator
     */
    public double update(double reference, double state) {
        double error;
        double derivative;
        // check if we need to unwrap angle
        if (angleWrap) {
            error = angleWrap(reference - state);
        } else {
            error = reference - state;
        }
        // define deriative value
        derivative = (error - lastError) / timer.seconds();

        // forward euler integration
        integral += error * timer.seconds();
        if (integral > 1) {
            integral = 1;
        }
        else if (integral < -1) {
            integral = -1;
        }

        double output = (error * Kp) + (integral * Ki) + (derivative * Kd);

        timer.reset();
        lastError = error; 

        return output;
    }


    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }


}