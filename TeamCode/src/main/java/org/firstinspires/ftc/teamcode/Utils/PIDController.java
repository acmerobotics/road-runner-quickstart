package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private final ElapsedTime period = new ElapsedTime();
    public double targetValue = 0;
    public double lastTargetValue = 0;
    public double maxOutput = 0.6;
    // Alpha is the extinction coefficient.  It must be between (0.0, 1.0]
    //   * 0 means that new values are ignored
    //   * 1 means that the previous values don't matter at all (no averaging).
    public double alpha = 0;

    // These follow the standard definitions
    public double p;
    public double i;
    public double d;
    public double maxIntegralSum;
    public double chillFactor = 1;
    public double error;

    private double lastError = 0;
    private double integratedError = 0.0;
    private double averagedDerivative = 0.0;
    /**
     * This PID Controller drives a motor using PID control as specified.  The function is
     * implemented using the following equation (approximately):
     * <p>
     * TargetPower = pValue * err() + iValue * integratedError + dValue * averagedDerivative
     * <p>
     * Notes:
     *  * integratedError is calculated as a proper integral (error * delta_time)
     *  * averagedDerivative is an exponential weighted moving average (EWMA) of the derivative of error
     *    See <a href="https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average">Wikipedia for EWMA definitions</a>
     * <p>
     * See <a href="https://en.wikipedia.org/wiki/PID_controller">Wikipedia on PID Controllers</a>.
     *
     * @param pValue The value for p
     * @param iValue The value for i
     * @param dValue The value for d
     */
    public PIDController(double pValue, double iValue, double dValue) {
        this.p = pValue;
        this.d = dValue;
        this.i = iValue;
        if (iValue ==0)
        {
            this.maxIntegralSum = 0;
        }
        else
        {
            this.maxIntegralSum = 0.25 / iValue;
        }
    }

    /**
     * Clamp a float value between upper and lower bounds.
     *
     * @param val the value
     * @param min lower bound
     * @param max upper bound
     * @return val clamped between min and max.
     */
    public static float clamp(float val, float min, float max) {
        return Math.max(min, Math.min(max, val));
    }

    /**
     * Clamp a double value between upper and lower bounds.
     *
     * @param val the value
     * @param min lower bound
     * @param max upper bound
     * @return val clamped between min and max.
     */
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    /**
     * Call this function inside the loop to update the PID calculations and set the optimal
     * motor power.
     */
    public double update(double measurement, double feedForward) {
        // Calculate how long since the last update and reset the timer.
        double timeStep = period.seconds();
        period.reset();

        // Store thisError because we'll use it several times.
        double thisError = measuredError(measurement);
        // Store thisDerivative because it makes the later equations easier to understand
        double thisDerivative = (thisError - lastError) / timeStep;

        // Only start integrating when we get close so large moves don't over-weight the error.
        integratedError = integratedError + thisError * timeStep;

        integratedError = clamp(integratedError,-maxIntegralSum,maxIntegralSum);

        averagedDerivative = thisDerivative * alpha + averagedDerivative * (1 - alpha);

        lastError = thisError;

        double powerCalc = p * thisError + i * integratedError + d * averagedDerivative + feedForward;
        return clamp(powerCalc, -maxOutput, maxOutput);
        /** Calculations for our custom PID control **/
    }

    public double update(double measurement) {
        return update(measurement, 0);
    }

    public double updateWithSpecificError(double error, double feedForward) {
        double timeStep = period.seconds();
        period.reset();

        // Store thisError because we'll use it several times.
        // Store thisDerivative because it makes the later equations easier to understand
        double thisDerivative = (error - lastError) / timeStep;

        // Only start integrating when we get close so large moves don't over-weight the error.
        integratedError = integratedError + error * timeStep;

        integratedError = clamp(integratedError,-maxIntegralSum,maxIntegralSum);

        averagedDerivative = thisDerivative * alpha + averagedDerivative * (1 - alpha);

        lastError = error;

        double powerCalc = p * error + i * integratedError + d * averagedDerivative + feedForward;
        return clamp(powerCalc, -maxOutput, maxOutput);
    }

    public double measuredError(double measurement) {
        return (targetValue - measurement);
    }

    public void setTargetValue(double targetValue) {
        this.lastTargetValue = this.targetValue;
        this.targetValue = targetValue;
        integratedError = 0.0;
    }

    public double getTargetValue() {
        return targetValue;
    }

    public void setIntegralTime(double time) {
        this.i = this.p / time;
    }

    public void setDerivativeTime(double time) {
        this.d = this.p * time;
    }

}
