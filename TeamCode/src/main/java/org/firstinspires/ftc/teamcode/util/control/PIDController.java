package org.firstinspires.ftc.teamcode.util.control;

import com.qualcomm.robotcore.util.RobotLog;

/**
 * This is NOT an opmode.
 *
 * THis Class is used to define a PID controller of any kind
 *
 */

public class PIDController
{
    private double lastError_;
    private double setPoint_;
    private double errorSum_;
    private double kp_;
    private double ki_;
    private double kd_;
    private long lastTime_;
    private boolean rot;
    public double error;

    /* Public OpMode members. */
    private PIDController() {}

    /* Constructor */
    public PIDController(double setPoint, double kp, double ki, double kd, boolean rot){
        setPoint_ = setPoint;
        lastError_ = 0;
        lastTime_ = System.currentTimeMillis();
        errorSum_ = 0;
        this.rot = rot;

        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    public float update(double newInput){

        long time = System.currentTimeMillis();
        long period = time - lastTime_;


        if(rot){
            error  = norm(setPoint_ - newInput);
        }else{
            error  = setPoint_ - newInput;
        }

        if ((int)Math.signum(lastError_) != (int) Math.signum(error)) {
            errorSum_ = 0;
        }

        errorSum_ +=  (error * period);
        double derError = (error - lastError_) / period;

        double output = (kp_ * error) + (ki_ * errorSum_) + (kd_ * derError);

        RobotLog.i("PID error is " + error + "; PID output is " + output + "; errorsum is " + errorSum_);

        lastError_ = error;
        lastTime_ = time;
        return (float) output;
    }

    public static double norm(double angle) {
        // return angle %TAU - Math.PI;
        angle = angle % 360.0f;

        angle = (angle + 360.0f) % 360.0f;

        if (angle > 180.0f) {
            angle -= 360.0f;
        }
        return angle;

    }
}