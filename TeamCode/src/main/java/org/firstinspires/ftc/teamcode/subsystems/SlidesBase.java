package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.aimrobotics.aimlib.control.FeedforwardController;
import com.aimrobotics.aimlib.control.LowPassFilter;
import com.aimrobotics.aimlib.control.PIDController;
import com.aimrobotics.aimlib.util.Mechanism;
import com.aimrobotics.aimlib.control.SimpleControlSystem;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class SlidesBase extends Mechanism {

    private DcMotorEx leftSlide;
    private DcMotorEx rightSlide;

    private DcMotorEx activeEncoderMotor;
    private double lastActiveEncoderPosition;

    String leftSlideName;
    String rightSlideName;

    DcMotorSimple.Direction leftMotorDirection;
    DcMotorSimple.Direction rightMotorDirection;

    double activeTargetPosition = 0;

    PIDController pidController;
    FeedforwardController feedforwardController;
    LowPassFilter lowPassFilter;
    SimpleControlSystem controlSystem;

    public static final double PROXIMITY_THRESHOLD = 30
            ;
    private static final double CURRENT_THRESHOLD = 5000;

    /**
     * Constructor for the slides base
     * @param leftSlideName the name of the left slide motor
     * @param rightSlideName the name of the right slide motor
     * @param leftMotorDirection the direction of the left motor
     * @param rightMotorDirection the direction of the right motor
     * @param kP the proportional gain
     * @param kI the integral gain
     * @param kD the derivative gain
     * @param derivativeLowPassGain the derivative low pass gain
     * @param integralSumMax the maximum integral sum
     * @param kV the velocity feedforward gain
     * @param kA  the acceleration feedforward gain
     * @param kStatic the static feedforward gain
     * @param kCos the cosine feedforward gain (Only this or kG should be used)
     * @param kG the gravity feedforward gain (Only this or kCos should be used)
     * @param lowPassGain the low pass gain
     */
    public SlidesBase(String leftSlideName, String rightSlideName, DcMotorSimple.Direction leftMotorDirection, DcMotorSimple.Direction rightMotorDirection, double kP, double kI, double kD, double derivativeLowPassGain, double integralSumMax, double kV, double kA, double kStatic, double kCos, double kG, double lowPassGain) {
        this.leftSlideName = leftSlideName;
        this.rightSlideName = rightSlideName;
        this.leftMotorDirection = leftMotorDirection;
        this.rightMotorDirection = rightMotorDirection;
        pidController = new PIDController(kP, kI, kD, derivativeLowPassGain, integralSumMax);
        feedforwardController = new FeedforwardController(kV, kA, kStatic, kCos, kG);
        lowPassFilter = new LowPassFilter(lowPassGain);
        controlSystem = new SimpleControlSystem(pidController, feedforwardController, lowPassFilter);
    }

    /**
     * Initialize the slides base
     * @param hwMap references the robot's hardware map
     */
    @Override
    public void init(HardwareMap hwMap) {
        leftSlide = hwMap.get(DcMotorEx.class, leftSlideName);
        rightSlide = hwMap.get(DcMotorEx.class, rightSlideName);
        setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftSlide.setDirection(leftMotorDirection);
        rightSlide.setDirection(rightMotorDirection);
        activeEncoderMotor = leftSlide;
        lastActiveEncoderPosition = 0;
    }

    /**
     * Set the mode of the slides
     * @param mode the mode to set the slides to
     */
    public void setMode(DcMotorEx.RunMode mode) {
        leftSlide.setMode(mode);
        rightSlide.setMode(mode);
    }

    /**
     * Set the zero power behavior of the slides
     * @param behavior the zero power behavior to set the slides to
     */
    private void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior behavior) {
        leftSlide.setZeroPowerBehavior(behavior);
        rightSlide.setZeroPowerBehavior(behavior);
    }

    /**
     * Set the power of the slides and update the last position
     * Only updates the last position when the slides are running vs. every loop
     * Slides will likely be running every loop
     * @param power the power to set the slides to
     */
    public void setPower(double power) {
        leftSlide.setPower(power);
        rightSlide.setPower(power);
        updateLastPosition();
    }

    /**
     * Stop the slides and set to 0 power
     */
    public void stop() {
        setPower(0);
    }

    /**
     * Update the power of the slides based on the control system's output
     */
    public void update() {
        double power = getTargetOutputPower();
        setPower(power);
    }

    /**
     * Hold the position of the slides
     */
    public void holdPosition() {
        setTargetPosition(getLastPosition());
        update();
    }

    /**
     * Set the target position for the slides
     * @param targetPosition the target position for the slides
     */
    public void setTargetPosition(double targetPosition) {
        activeTargetPosition = targetPosition;
        controlSystem.setTarget(activeTargetPosition);
    }

    /**
     * Get the output power of the slides based on the system update for the active target position
     * @return the output power for the slide motors
     */
    private double getTargetOutputPower() {
        return controlSystem.update(getCurrentPosition());
    }

    /**
     * Check if the slides are at the target position
     * @return true if the slides are within the proximity threshold of the target position
     */
    public boolean isAtTargetPosition() {
        return Math.abs(getCurrentPosition() - activeTargetPosition) < PROXIMITY_THRESHOLD;
    }

    /**
     * Get the current position of the slides
     * @return the current position of the slides
     */
    public double getCurrentPosition() {
        return activeEncoderMotor.getCurrentPosition();
    }

    /**
     * Set the last active encoder position to the encoder motor's current position
     */
    public void updateLastPosition() {
        lastActiveEncoderPosition = activeEncoderMotor.getCurrentPosition();
    }

    /**
     * Get the last active encoder position
     * @return the last active encoder position
     */
    public double getLastPosition() {
        return lastActiveEncoderPosition;
    }

    /**
     * Check if the slides are over the current threshold
     * @return true if the slides are over the current threshold
     */
    public boolean currentSpikeDetected() {
        return activeEncoderMotor.getCurrent(CurrentUnit.MILLIAMPS) > CURRENT_THRESHOLD;
    }
}

