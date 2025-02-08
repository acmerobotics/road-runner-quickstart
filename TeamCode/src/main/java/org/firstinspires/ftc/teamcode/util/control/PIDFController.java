package org.firstinspires.ftc.teamcode.util.control;

import com.qualcomm.robotcore.util.ElapsedTime;


public class PIDFController {

    private ElapsedTime timer = new ElapsedTime(); // timer for calculating change in time

    private boolean hasStarted = false; // has the timer started
    private double integralSum = 0; // sum of the integral
    private double previousError = 0; // previous error

    private final double kP; // proportional constant
    private final double kI; // integral constant
    private final double kD; // derivative constant
    private double integralSumMax; // maximum value of the integral sum

    private double kV; // velocity proportional constant
    private double kA; // acceleration proportional constant
    private double kStatic; // static friction constant
    private double kCos; // cosine compensation constant (Degrees)
    private double kG; // gravity compensation constant

    private double targetPos;

    public enum FeedforwardType {
        NONE, ROTATIONAL, LINEAR
    }

    private FeedforwardType activeFFType = FeedforwardType.NONE;

    private double length = 0;

    public PIDFController(double kP, double kI, double kD, double integralSumMax, double kV, double kA, double kStatic, double kCos, double kG, FeedforwardType feedforwardType) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.integralSumMax = integralSumMax;
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
        this.kCos = kCos;
        this.kG = kG;
        activeFFType = feedforwardType;
        reset();
    }

    public void setTargetPos(double targetPos) {
        this.targetPos = targetPos;
    }

    public double calculateOutput(double target, double referencePosition, double referenceVelocity, double referenceAcceleration) {
        double dt = getDT();
        double error = target - referencePosition;
        double derivative = calculateDerivative(error, dt);
        integrate(error, dt);
        previousError = error;
        return (kP * error) + (kI * integralSum) + (kD * derivative) + calculateFeedForward(referenceVelocity, referenceAcceleration);
    }

    /**
     * Gets the change in time since the last call
     * @return change in time
     */
    private double getDT() {
        if (!hasStarted) {
            timer.reset();
            hasStarted = true;
        }
        double dt = timer.seconds();
        timer.reset();
        return dt;
    }


    /**
     * Calculates the derivative of the error then filters the result
     * @param error error of the system
     * @param dt change in time
     * @return filtered derivative of the error
     */
    private double calculateDerivative(double error, double dt) {
        double derivative = (error - getPreviousError()) / dt;
        return derivative;
    }

    /**
     * Integrates the error and caps the sum. Resets the sum if the error crosses zero
     * @param error error of the system
     * @param dt change in time
     */
    private void integrate(double error, double dt) {
        if (hasErrorCrossedZero(error)) {
            integralSum = 0;
        }
        integralSum += error * dt;
        if (Math.abs(integralSum) > integralSumMax) {
            integralSum = Math.signum(integralSum) * integralSumMax;
        }
    }

    /**
     * Checks if the error has crossed zero
     * @param error error of the system
     * @return true if the error has crossed zero in the last loop
     */
    private boolean hasErrorCrossedZero(double error) {
        return (error > 0 && getPreviousError() < 0) || (error < 0 && getPreviousError() > 0);
    }

    /**
     * Gets the previous error
     * @return previous error
     */
    private double getPreviousError() {
        return previousError;
    }

    private void reset() {
        hasStarted = false;
        integralSum = 0;
        previousError = 0;
    }

    public void setLength(double length) {
        this.length = length;
    }

    private double calculateFeedForward(double referenceVelocity, double referenceAcceleration) {
        double ffReturn = (kV * referenceVelocity) + (kA * referenceAcceleration);
        switch (activeFFType) {
            case ROTATIONAL:
                return ffReturn + (Math.cos(Math.toRadians(targetPos)) * kCos * length);
            case LINEAR:
                return ffReturn + kG;
            default:
                return ffReturn;
        }
    }
}
