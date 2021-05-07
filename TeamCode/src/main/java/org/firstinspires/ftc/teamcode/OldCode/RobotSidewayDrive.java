// ////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2018 ViBoTs Vines High School FTC-11341.
// ////////////////////////////////////////////////////////////////////////////
// SCM: $Id: RobotSidewayDrive.java 191 2019-11-15 19:35:45Z veeral $
// ////////////////////////////////////////////////////////////////////////////
package org.firstinspires.ftc.teamcode.OldCode;

/**
 * Sidways motion capable robot drive interface.
 * A robot-drive that enables sideways movements must implement this interface.
 * Drives using wheels such as Mecanum wheels typically implement this interface.
 *
 * Note that due to the mechanics of sideways motion, the encoder values for sideways
 * motion may be much different than the linear forward and backward motion.
 *
 * @author Harshal Bharatia
 */
public interface RobotSidewayDrive
{
    /**
     * Move sideways to the left.
     *
     * @param power Percentage power that determines the speed of traversal
     *  with absolute value of the power, 1.0 for full speed and
     *  0 stopping the drive's motion.
     */
    public void sidewaysLeft(double power);

    /**
     * Move all motors to cause sideways motion towards left of drive until
     * the encoder position of all drives reach the specified value. Absolute
     * power value determines robot speed. Cumulative encoder mode is
     * supported due to complex management of such cumulative values for
     * individual motors.
     *
     * @param power Percentage power that determines the speed of traversal.
     * @param encoderTarget  The target encoder value that must be reached by
     *  each motor in the drive.
     */
    public void sidewaysLeft(double power, int encoderTarget);

    /**
     * Move sideways to the right.
     *
     * @param power Percentage power that determines the speed of traversal
     *  with absolute value of the power, 1.0 for full speed and
     *  0 stopping the drive's motion.
     */
    public void sidewaysRight(double power);

    /**
     * Move all motors to cause sideways motion towards right of drive until
     * the encoder position of all drives reach the specified value. Absolute
     * power value determines robot speed. Cumulative encoder mode is
     * supported due to complex management of such cumulative values for
     * individual motors.
     *
     * @param power Percentage power that determines the speed of traversal.
     * @param encoderTarget  The target encoder value that must be reached by
     *  each motor in the drive.
     */
    public void sidewaysRight(double power, int encoderTarget);
}