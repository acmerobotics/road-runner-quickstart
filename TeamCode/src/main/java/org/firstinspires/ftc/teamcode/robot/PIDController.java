package org.firstinspires.ftc.teamcode.robot;
import com.qualcomm.robotcore.util.Range;


public class PIDController {

    private double target;
    private double kP, kI, kD;
    private double proportional, integral, derivative;
    private boolean shouldReset;

    private double previousTime, previousError;

    private double lowerInputBound = Double.NEGATIVE_INFINITY, higherInputBound = Double.POSITIVE_INFINITY;
    private double lowerOutputBound = Double.NEGATIVE_INFINITY, higherOutputBound = Double.POSITIVE_INFINITY;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        shouldReset = true;
    }

    public void setPIDValues(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double getTarget() {
        return target;
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public void setInputBounds(double lowerInputBound, double higherInputBound) {
        this.lowerInputBound = lowerInputBound;
        this.higherInputBound = higherInputBound;
    }

    public void setOutputBounds(double lowerOutputBound, double higherOutputBound) {
        this.lowerOutputBound = lowerOutputBound;
        this.higherOutputBound = higherOutputBound;
    }

    public double getLowerInputBound(){
        return this.lowerInputBound;
    }

    public double getUpperInputBound(){
        return this.higherInputBound;
    }


    public void reset() {
        shouldReset = true;
    }

    public double update(double value) {
        value = Range.clip(value, lowerInputBound, higherInputBound);

        double error = value - target;

        return updateWithError(error);
    }

    public double updateWithError(double error) {
        if (Double.isNaN(error) || Double.isInfinite(error))
            return 0;

        proportional = kP * error;

        double currentTime = System.currentTimeMillis() / 1000.0;

        if (shouldReset) {
            shouldReset = false;
            integral = 0;
            derivative = 0;
            previousError = error;
        } else {
            double dT = currentTime - previousTime;

            integral += kI * error * dT;

            derivative = kD * (error - previousError) / dT;
        }

        previousTime = currentTime;
        previousError = error;

        double correction = proportional + integral + derivative;

        return Math.signum(correction) * Range.clip(Math.abs(correction),
                lowerOutputBound, higherOutputBound);
    }
}
