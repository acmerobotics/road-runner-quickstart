package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * A utility class to contain two <code>DcMotor</code>s and use them together.
 * Contains most methods of <code>DcMotor</code>.
 *
 */

public class dualMotor {
    private String noMotorEx = "Ninevolt: No motors remaining!";
    private DcMotor motor1;
    private DcMotor motor2;
    private boolean isSingleMotor = false;

    /**
     * Constructs a DcMotorPair containing and controlling at least one motor.
     * @param motor1 The first motor to be controlled by this pair
     * @param motor2 The second motor to be controlled by this pair
     * @throws Exception Thrown when no motors are provided
     */
    public dualMotor(DcMotorEx motor1, DcMotorEx motor2) throws Exception {
        this.motor1 = motor1;
        this.motor2 = motor2;
        if (motor1 == null ^ motor2 == null) {
            isSingleMotor = true;
        } else if (motor1 == null) {
            throw new Exception(noMotorEx);
        }
    }

    /**
     * Constructs a DcMotorPair controlling one motor.
     * @param motor1 - The DcMotor to be controlled by this motor pair.
     */
    public dualMotor(DcMotor motor1) {
        this.motor1 = motor1;
        isSingleMotor = true;
    }

    /**
     * Gets the first DcMotor controlled by this motor pair.
     * @return The first motor controlled by this pair.
     */
    public DcMotor getMotor1() {
        return motor1;
    }

    /**
     * Gets the second motor controlled by this motor pair.
     * @return The second motor controlled by this pair.
     */
    public DcMotor getMotor2() {
        return motor2;
    }

    /**
     * Sets the first motor controlled by this motor pair.
     * @param motor1 The motor to be controlled by this pair.
     * @throws Exception Thrown if no motors are remaining in this pair.
     */
    public void setMotor1(DcMotor motor1)throws Exception {
        this.motor1 = motor1;
        if (this.motor1 == null && motor2 != null) {
            isSingleMotor = true;
        } else if (this.motor1 != null && motor2 != null) {
            isSingleMotor = false;
        } else {
            throw new Exception(noMotorEx);
        }

    }

    /**
     * Sets the second motor controlled by this motor pair.
     * @param motor2 The motor to be controlled by this pair.
     * @throws Exception Thrown if no motors are remaining in this pair.
     */
    public void setMotor2(DcMotor motor2) throws Exception {
        this.motor2 = motor2;
        if (motor1 != null && this.motor2 == null) {
            isSingleMotor = true;
        } else if (motor1 != null && this.motor2 != null) {
            isSingleMotor = false;
        } else {
            throw new Exception(noMotorEx);
        }
    }

    /**
     * Returns true if this pair only controls one motor.
     * @return true if this pair only controls one motor.
     */
    public boolean isSingleMotor() {
        return isSingleMotor;
    }


    /**
     * Sets the power of all motors in this pair.
     * @param power The power to set to each motor.
     */
    public void setPower(double power) {
        if(motor1 != null)
            motor1.setPower(power);
        if(motor2 != null)
            motor2.setPower(power);
    }

    /**
     * Sets the direction of all motors in the pair.
     * Calls <code>DcMotor.setDirection(Direction)</code> on each motor.
     * @param direction The <code>DcMotor.Direction</code> to set the direction of the motors to.
     */
    public void setDirection(DcMotor.Direction direction) {
        if(motor1 != null)
            motor1.setDirection(direction);
        if(motor2 != null)
            motor2.setDirection(direction);
    }

    /**
     * Sets the current run mode of all motors in the pair.
     * @param runMode The run mode to set for each motor.
     */
    public void setMode(DcMotor.RunMode runMode) {
        if(motor1 != null)
            motor1.setMode(runMode);
        if(motor2 != null) {
            motor2.setMode(runMode);
        }
    }

    /**
     * Sets the target position of all motors in the pair.
     * @param targetPosition The target position (in encoder ticks) to set each
     *                       motor's target position to.
     */
    public void setTargetPosition(int targetPosition) {
        if(motor1 != null)
            motor1.setTargetPosition(targetPosition);
        if(motor2 != null) {
            motor2.setTargetPosition(targetPosition);
        }
    }

    /**
     * Gets the current position of the first motor if there are two motors,
     * otherwise, gets the current position of the single motor.
     * @return The position of the motor(s) in encoder ticks.
     * @throws Exception Thrown when no motors were found in the current pair.
     */
    public int getCurrentPosition() throws Exception {
        if(motor1 != null && motor2 != null) {
            return motor1.getCurrentPosition();
        }
        if(motor1 != null)
            return motor1.getCurrentPosition();
        else if(motor2 != null)
            return motor2.getCurrentPosition();
        else
            throw new Exception(noMotorEx);
    }

}