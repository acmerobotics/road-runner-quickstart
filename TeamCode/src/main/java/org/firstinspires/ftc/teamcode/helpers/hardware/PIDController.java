package org.firstinspires.ftc.teamcode.helpers.hardware;

import androidx.annotation.Nullable;

/**
 * PID controller with various feedforward components and wrap-around support.
 */
public class PIDController {
    public interface FeedforwardFun {
        double compute(double position, @Nullable Double velocity);
    }

    public enum WrapMode {
        SHORT,    // Wrap to the shortest path
        POSITIVE, // Always move in the positive direction
        NEGATIVE  // Always move in the negative direction
    }

    // PID coefficients
    private double kP;
    private double kI;
    private double kD;

    private final double kV;
    private final double kA;
    private final double kStatic;
    private final FeedforwardFun kF;

    private double errorSum;
    private long lastUpdateTs;

    private boolean inputBounded;
    private double minInput, maxInput;

    private boolean outputBounded;
    private double minOutput, maxOutput;

    private WrapMode wrapMode = WrapMode.SHORT; // Default wrap mode

    // Integral windup limit
    private double maxIntegral = 1.0; // Adjust this limit based on your system

    // PID term components for telemetry
    private double pTerm;
    private double iTerm;
    private double dTerm;

    // Error terms for derivative calculation
    private double lastError;
    private double errorDerivative;

    /**
     * Target position (that is, the controller setpoint).
     */
    public double targetPosition;

    /**
     * Target velocity.
     */
    public double targetVelocity;

    /**
     * Target acceleration.
     */
    public double targetAcceleration;

    /**
     * Creates a PID controller with specified PID coefficients and feedforward components.
     *
     * @param kP     proportional gain
     * @param kI     integral gain
     * @param kD     derivative gain
     * @param kV     feedforward velocity gain
     * @param kA     feedforward acceleration gain
     * @param kStatic additive feedforward constant
     * @param kF     custom feedforward function
     */
    public PIDController(
            double kP,
            double kI,
            double kD,
            double kV,
            double kA,
            double kStatic,
            FeedforwardFun kF
    ) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
        this.kF = kF;
    }

    /**
     * Creates a PID controller with specified PID coefficients.
     *
     * @param kP proportional gain
     * @param kI integral gain
     * @param kD derivative gain
     */
    public PIDController(double kP, double kI, double kD) {
        this(kP, kI, kD, 0, 0, 0, (x, v) -> 0);
    }

    /**
     * Sets the PID coefficients.
     *
     * @param kP proportional gain
     * @param kI integral gain
     * @param kD derivative gain
     */
    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /**
     * Sets the maximum integral windup limit.
     *
     * @param maxIntegral maximum integral value
     */
    public void setMaxIntegral(double maxIntegral) {
        this.maxIntegral = maxIntegral;
    }

    /**
     * Sets bounds on the input of the controller. When computing the error, the min and max are
     * treated as the same value, effectively creating a circular range.
     *
     * @param min minimum input
     * @param max maximum input
     */
    public void setInputBounds(double min, double max) {
        if (min < max) {
            inputBounded = true;
            minInput = min;
            maxInput = max;
        }
    }

    /**
     * Sets bounds on the output of the controller.
     *
     * @param min minimum output
     * @param max maximum output
     */
    public void setOutputBounds(double min, double max) {
        if (min < max) {
            outputBounded = true;
            minOutput = min;
            maxOutput = max;
        }
    }

    /**
     * Sets the wrap mode for the controller.
     *
     * @param wrapMode the desired wrap mode
     */
    public void setWrapMode(WrapMode wrapMode) {
        this.wrapMode = wrapMode;
    }

    private double positiveModulo(double value, double modulo) {
        double result = value % modulo;
        if (result < 0) {
            result += modulo;
        }
        return result;
    }

    private double wrapErrorToRange(double error, double minRange, double maxRange) {
        double range = maxRange - minRange;
        error = (error - minRange) % range + minRange;
        if (error < minRange) {
            error += range;
        }
        return error;
    }

    private double getPositionError(double measuredPosition) {
        double error = targetPosition - measuredPosition;

        if (inputBounded) {
            final double inputRange = maxInput - minInput;

            switch (wrapMode) {
                case SHORT:
                    // Wrap error to [-inputRange/2, inputRange/2)
                    error = wrapErrorToRange(error, -inputRange / 2, inputRange / 2);
                    break;

                case POSITIVE:
                    // Always move in the positive direction
                    error = positiveModulo(error, inputRange);
                    break;

                case NEGATIVE:
                    // Always move in the negative direction
                    error = positiveModulo(error, inputRange);
                    if (error > 0) {
                        error -= inputRange;
                    }
                    break;
            }
        }

        return error;
    }

    /**
     * Run a single iteration of the controller.
     *
     * @param timestamp        measurement timestamp as given by {@link System#nanoTime()}
     * @param measuredPosition measured position (feedback)
     * @param measuredVelocity measured velocity
     */
    public double update(
            long timestamp,
            double measuredPosition,
            @Nullable Double measuredVelocity
    ) {
        final double error = getPositionError(measuredPosition);

        if (lastUpdateTs == 0) {
            lastError = error;
            lastUpdateTs = timestamp;
            return 0;
        }

        final double dt = (timestamp - lastUpdateTs) / 1e9; // Convert nanoseconds to seconds

        if (dt <= 0) {
            // Avoid division by zero or negative time delta
            return 0;
        }

        errorSum += error * dt;

        // Implement integral windup protection
        if (kI != 0) {
            errorSum = Math.max(-maxIntegral / kI, Math.min(errorSum, maxIntegral / kI));
        } else {
            errorSum = 0;
        }

        errorDerivative = (error - lastError) / dt;

        lastError = error;
        lastUpdateTs = timestamp;

        double velError;
        if (measuredVelocity == null) {
            velError = errorDerivative;
        } else {
            velError = targetVelocity - measuredVelocity;
        }

        // Calculate individual PID terms
        pTerm = kP * error;
        iTerm = kI * errorSum;
        dTerm = kD * velError;

        double baseOutput = pTerm + iTerm + dTerm +
                kV * targetVelocity + kA * targetAcceleration +
                kF.compute(measuredPosition, measuredVelocity);

        double output = baseOutput;

        // Add kStatic if output is significant
        if (Math.abs(baseOutput) > 1e-6) {
            output += Math.copySign(kStatic, baseOutput);
        }

        if (outputBounded) {
            output = Math.max(minOutput, Math.min(output, maxOutput));
        }

        // Prevent integral windup when output is saturated
        if (output == minOutput || output == maxOutput) {
            // Do not accumulate error if the output is saturated
            errorSum -= error * dt;
            iTerm = kI * errorSum; // Update iTerm after adjusting errorSum
        }

        return output;
    }

    public double update(
            long timestamp,
            double measuredPosition
    ) {
        return update(timestamp, measuredPosition, null);
    }

    public double update(
            double measuredPosition
    ) {
        return update(System.nanoTime(), measuredPosition, null);
    }

    /**
     * Reset the controller's integral sum.
     */
    public void reset() {
        errorSum = 0;
        lastError = 0;
        lastUpdateTs = 0;
    }

    // Getters for PID term components
    public double getPTerm() {
        return pTerm;
    }

    public double getITerm() {
        return iTerm;
    }

    public double getDTerm() {
        return dTerm;
    }

    public double getLastError() {
        return lastError;
    }

    public double getErrorSum() {
        return errorSum;
    }

    public double getErrorDerivative() {
        return errorDerivative;
    }
}
