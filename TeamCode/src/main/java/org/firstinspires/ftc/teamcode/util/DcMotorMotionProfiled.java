package org.firstinspires.ftc.teamcode.util;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * DcMotorMotinoProfiled class is an implementation of a DcMotor that adds a motion profile
 * to the setTargetPosition method. This improves the motor's positional movement.
 * The class also adds other methods to add additional functionality to a DcMotor
 *
 * @author Nate Schmelkin
 * Inspiration Paul Serbanescu Stuy Fission #310
 */

public class DcMotorMotionProfiled implements DcMotorSimple {

    /**
     * Motor
     */
    protected DcMotorEx motor;

    /**
     * Empirical output constants
     */
    private Double OUTPUT_RADIUS;
    private Double GEAR_RATIO;
    private Double TICKS_PER_REV;

    /**
     * Motion constraints
     */
    private Double MAX_VEL;
    private Double MAX_ACCEL;

    private MotionProfile motionProfile;
    private final ElapsedTime profileTimer = new ElapsedTime();


    /**
     * Used with linear slides to account for gravity when decreasing in position
     */
    private Double RETRACTION_MULTIPLIER;

    /**
     * PID controller for motion profile
     */
    private PIDFController PIDcontroller;

    /**
     * Initializes the motor and sets it to the proper mode and stop behavior
     *
     * @param hwMap references hardware map
     * @param motorName references config name
     */
    public void initialize(HardwareMap hwMap, String motorName) {
        motor = hwMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    /**
     * Sets the velocity and acceleration constraints of the motor
     *
     * @param MAX_VEL (in/s)
     * @param MAX_ACCEL (in/s^2)
     */
    public void setMotionConstraints(double MAX_VEL, double MAX_ACCEL) {
        this.MAX_VEL = MAX_VEL;
        this.MAX_ACCEL = MAX_ACCEL;
    }

    /**
     * Sets PID gains used by PIDFController
     *
     * @param kP proportional gain
     * @param kI integral gain
     * @param kD derivative gain
     * @param kF kF
     */
    public void setPIDCoefficients(double kP, double kI, double kD, double kF) {
        PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);
        this.PIDcontroller = new PIDFController(coeffs, 0, 0, 0, (position, velocity) -> kF);
    }


    /**
     * Sets retraction multiplier
     *
     * @param multiplier retraction multiplier
     */
    public void setRetractionMultiplier(double multiplier) {
        this.RETRACTION_MULTIPLIER = multiplier;
    }

    /**
     * Sets output constants to be used in {@link #encoderTicksToInches(double) encoderTicksToInches}
     *
     * @param OUTPUT_RADIUS is the radius of the wheel/spool or any other output attached to the motor
     * @param GEAR_RATIO is the gear ratio of input (motor) speed to output (output) speed
     * @param TICKS_PER_REV is the number of ticks per one revolution of the motor. Can be found on motor's website
     */
    public void setOutputConstants(double OUTPUT_RADIUS, double GEAR_RATIO, double TICKS_PER_REV) {
        this.OUTPUT_RADIUS = OUTPUT_RADIUS;
        this.GEAR_RATIO = GEAR_RATIO;
        this.TICKS_PER_REV = TICKS_PER_REV;

    }

    private double encoderTicksToInches(double ticks) {
        try {
            return OUTPUT_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
        } catch (NullPointerException e) {
            RobotLog.setGlobalErrorMsg("%s output constants not set. make sure to setOutputConstants(double, double, double) "
                    + getClass().getSimpleName());
        }
        return 0.0;
    }

    /**
     * Returns motor position (inches)
     *
     * @return inches away from starting position
     */
    public double getPosition() {
        return encoderTicksToInches(motor.getCurrentPosition());
    }

    /**
     * Returns motor velocity
     *
     * @return motor velocity (in/s)
     */
    public double getVelocity() {
        return encoderTicksToInches(motor.getVelocity());
    }

    /**
     *
     * @param targetPosition (inches)
     * @param maxVel maximum velocity motor can reach in motion profile
     * @param maxAccel maximum acceleration motor can reach in motor profile
     * @return motion profile from start position to goal
     */
    public MotionProfile generateMotionProfile(double targetPosition, double maxVel, double maxAccel) {
        MotionState start = new MotionState(getPosition(), getVelocity(), 0, 0);
        MotionState goal = new MotionState(targetPosition, 0, 0, 0);

        try {
            return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, maxVel, maxAccel);
        } catch (NullPointerException e) {
            RobotLog.setGlobalErrorMsg("%s motion constraints not set. make sure to setMotionConstraints(double, double) "
                    + getClass().getSimpleName());
        }
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, 0, 0);
    }

    /**
     * Uses generateMotionProfile(double, Double, Double) to set motor target position using motion profile
     *
     * @param targetPosition (inches)
     */
    public void setTargetPosition(double targetPosition) {
        if (targetPosition < getPosition()) { motionProfile = generateMotionProfile(targetPosition, MAX_VEL * RETRACTION_MULTIPLIER, MAX_ACCEL * RETRACTION_MULTIPLIER); }
        else { motionProfile = generateMotionProfile(targetPosition, MAX_VEL, MAX_ACCEL); }
        profileTimer.reset();
    }

    /**
     * Overrides DcMotor setTargetPosition(int) method to use motion profile
     *
     * @param targetPosition (inches)
     */
    public void setTargetPosition(int targetPosition) {
        setTargetPosition((double) targetPosition);
    }

    /**
     * Call this function during the loop function of opMode so that the
     *  profile will properly function. Otherwise, the profile will not function
     */
    public void update() {
        MotionState state = motionProfile.get(profileTimer.seconds());

        PIDcontroller.setTargetPosition(state.getX());
        PIDcontroller.setTargetVelocity(state.getV());
        PIDcontroller.setTargetAcceleration(state.getA());

        double power = PIDcontroller.update(getPosition(), getVelocity());

        motor.setPower(power);
    }

    // =======================
    // pass thru functionality
    // =======================

    /**
     * Sets the direction of the motor
     *
     * @param direction direction of motor
     */
    @Override
    public void setDirection(Direction direction) {
        motor.setDirection(direction);
    }

    /**
     * Retrieves the current direction that the motor is operating at
     *
     * @return current direction the motor is operating at
     */
    @Override
    public Direction getDirection() {
        return motor.getDirection();
    }

    /**
     * Sets the power of the motor
     *
     * @param power the new power level of the motor [-1.0, 1.0]
     */
    @Override
    public void setPower(double power) {
        motor.setPower(power);
    }

    /**
     * Returns the current power level of the motor
     *
     * @return the current absolute value of the motor power [0.0, 1.0]
     */
    @Override
    public double getPower() {
        return motor.getPower();
    }

    /**
     * Returns the manufacturer of the motor
     *
     * @return Manufacturer of the motor
     */
    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    /**
     * Returns a string that describes the device.
     * NOTE: this is separate from the robot configuration
     *
     * @return device manufacturer and name
     */
    @Override
    public String getDeviceName() {
        return motor.getDeviceName();
    }

    /**
     * Returns connection information about device in readable format
     *
     * @return connection info
     */
    @Override
    public String getConnectionInfo() {
        return motor.getConnectionInfo();
    }

    /**
     * Returns version of device
     *
     * @return version of the device
     */
    @Override
    public int getVersion() {
        return motor.getVersion();
    }

    /**
     * Resets the device's configuration to what is expeted at the beginning of an OpMode.
     * Ex. sets direction to 'forward'
     */
    @Override
    public void resetDeviceConfigurationForOpMode() {
        motor.resetDeviceConfigurationForOpMode();
    }

    /**
     * Closes the device
     */
    @Override
    public void close() {

    }

    /**
     * Returns current used by this motor
     *
     * @param unit current units
     * @return current used by this motor
     */
    public double getCurrent(CurrentUnit unit) {
        return motor.getCurrent(unit);
    }

    public boolean profileDone() {
        return profileTimer.seconds() > motionProfile.duration();
    }
}
