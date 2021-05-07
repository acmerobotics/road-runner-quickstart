// ////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2018 ViBoTs Vines High School FTC-11341.
// ////////////////////////////////////////////////////////////////////////////
// SCM: $Id: RobotDrive.java 191 2019-11-15 19:35:45Z veeral $
// ////////////////////////////////////////////////////////////////////////////
package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Robot motor drive interface for robots with encoders.
 * This interface abstracts number of motors present in a robot's drive and
 * allows controlling the drive's motion using controls that involves only
 * left-motor(s), right-motor(s) or all motors.
 *
 * @author Harshal Bharatia
 */
public interface RobotDrive
{
    /**
     * Move all motors linearly to cause drive to traverse forwards or backwards.
     * This move is not encoder based and hence user must explicitly stop the
     * move.
     *
     * @param power Percentage power that determines the speed of traversal
     *  with -1.0 for full speed backwards, 1.0 for full speed forwards.
     *  0 stops the drive's motion.
     */
    public void moveAll(double power);

    /**
     * Move all motors linearly to run until the encoder position of all
     * drives reach the specified value. Values greater than current value
     * moves robot forward and values less than current value moves robot
     * backwards. Absolute power value determines robot speed.
     * Use cumulative to continue without resetting encoder of all motor(s)
     * to zero before starting motion.
     *
     * @param power Percentage power that determines the speed of traversal.
     * @param encoderTarget  The target encoder value that must be reached by
     *  each motor in the drive.
     * @param cumulative If true, the target encoder value is not reset to zero
     *  for each motor before the motion begins.
     */
    public void moveAll(double power, int encoderTarget, Boolean cumulative);

    /**
     * Move only left motor(s) linearly forwards or backwards.
     * This move is not encoder based and hence user must explicitly stop the
     * move.
     *
     * @param power Percentage power that determines the speed of traversal
     *  with -1.0 for full speed backwards, 1.0 for full speed forwards.
     *  0 stops the left motor(s) motion.
     */
    public void moveLeft(double power);

    /**
     * Move left motor(s) linearly to run until the encoder position of all
     * left motor(s) reach the specified value. Values greater than current value
     * moves left motor(s) forward and values less than current value moves left
     * motor(s) backwards. Absolute power value determines left motor(s) speed.
     * Use cumulative to continue without resetting encoder of all left motor(s)
     * to zero before starting motion.
     *
     * @param power Percentage power that determines the speed of traversal.
     * @param encoderTarget  The target encoder value that must be reached by
     *  each left motor in the drive.
     * @param cumulative If true, the target encoder value is not reset to zero
     *  for each motor before the motion begins
     */
    public void moveLeft(double power, int encoderTarget, Boolean cumulative);

    /**
     * Move only right motor(s) linearly forwards or backwards.
     * This move is not encoder based and hence user must explicitly stop the
     * move.
     *
     * @param power Percentage power that determines the speed of traversal
     *  with -1.0 for full speed backwards, 1.0 for full speed forwards.
     *  0 stops the right motor(s) motion.
     */
    public void moveRight(double power);

    /**
     * Move right motor(s) linearly to run until the encoder position of all
     * right motor(s) reach the specified value. Values greater than current value
     * moves right motor(s) forward and values less than current value moves right
     * motor(s) backwards. Absolute power value determines right motor(s) speed.
     * Use relative to reset encoder of all left motor(s) to zero before starting
     * motion.
     *
     * @param power Percentage power that determines the speed of traversal.
     * @param encoderTarget  The target encoder value that must be reached by
     *  each right motor in the drive.
     * @param cumulative If true, the target encoder value is not reset to zero
     *  for each motor before the motion begins
     */
    public void moveRight(double power, int encoderTarget, Boolean cumulative);

    /**
     * Prepare the drive to rotate.
     */
    public void prepareToRotate();
    /**
     * Update the power of ongoing motion of all motor(s) to specified value.
     *
     * @param newPower  Percentage power that determines the speed of traversal.
     *  For non-encoder assisted moves, use -1.0 for full speed backwards,
     *  1.0 for full speed forwards. For encoder assisted moves, use values from
     *  0.0 to 1.0 as direction is decided by the target encoder value.
     */
    public void updateAllPower(double newPower);

    /**
     * Update the power of ongoing motion of left motor(s) to specified value.
     *
     * @param newPower  Percentage power that determines the speed of traversal.
     *  For non-encoder assisted moves, use -1.0 for full speed backwards,
     *  1.0 for full speed forwards. For encoder assisted moves, use values from
     *  0.0 to 1.0 as direction is decided by the target encoder value.
     */
    public void updateLeftPower(double newPower);

    /**
     * Update the power of ongoing motion of right motor(s) to specified value.
     *
     * @param newPower  Percentage power that determines the speed of traversal.
     *  For non-encoder assisted moves, use -1.0 for full speed backwards,
     *  1.0 for full speed forwards. For encoder assisted moves, use values from
     *  0.0 to 1.0 as direction is decided by the target encoder value.
     */
    public void updateRightPower(double newPower);

    /**
     * Stop all motors.
     */
    public void stopAll();

    /**
     * Stop all left motor(s).
     */
    public void stopLeft();

    /**
     * Stop all right motor(s).
     */
    public void stopRight();

    /**
     * Reset the current encoder value for all motor(s).
     */
    public void resetAllEncoder();

    /**
     * Reset the current encoder value for left motor(s).
     */
    public void resetLeftEncoder();

    /**
     * Reset the current encoder value for right motor(s).
     */
    public void resetRightEncoder();

    /**
     * Check if any motor is still busy moving.
     *
     * @return True if atleast one motor is still busy moving.
     */
    public boolean isAnyBusy();

    /**
     * Check if any left motor is still busy moving.
     *
     * @return True if atleast one left motor is still busy moving.
     */
    public boolean isLeftBusy();

    /**
     * Check if any right motor is still busy moving.
     *
     * @return True if atleast one right motor is still busy moving.
     */
    public boolean isRightBusy();

    /**
     * Check if any motor has completed its task and therefore is not busy moving.
     *
     * @return True if atleast one motor has completed task and therefore is not busy moving.
     */
    public boolean isAnyComplete();

    /**
     * Check if any left motor has completed its task and therefore is not busy moving.
     *
     * @return True if atleast one motor has completed task and therefore is not busy moving.
     */
    public boolean isAnyLeftComplete();

    /**
     * Check if any left motor has completed its task and therefore is not busy moving.
     *
     * @return True if atleast one motor has completed task and therefore is not busy moving.
     */
    public boolean isAnyRightComplete();

    /**
     * Wait until all motor(s) have completed processing.
     *
     * @param preemptAnyComplete    Preempt the wait on any motor completion and stop rest.
     *                              If false, awaits completion of all motor(s).
     * @param monitor        Optional monitor to restrict operation's operating conditions.
     * @return  True if operation was preempted;
     */
    public boolean waitAllComplete(boolean preemptAnyComplete, ConditionMonitor monitor);

    /**
     * Wait until left motor(s) have completed processing.
     *
     * @param preemptAnyComplete    Preempt the wait on any motor completion and stop rest.
     *                              If false, awaits completion of all left motor(s).
     * @param monitor        Optional monitor to restrict operation's operating conditions.
     * @return  True if operation was preempted;
     */
    public boolean waitLeftComplete(boolean preemptAnyComplete, ConditionMonitor monitor);

    /**
     * Wait until right motor(s) have completed processing.
     *
     * @param preemptAnyComplete    Preempt the wait on any motor completion and stop rest.
     *                              If false, awaits completion of all right motor(s).
     * @param monitor        Optional monitor to restrict operation's operating conditions.
     * @return  True if operation was preempted;
     */
    public boolean waitRightComplete(boolean preemptAnyComplete, ConditionMonitor monitor);

    /**
     * Run as long as the specified monitor permits such run.
     *
     * @param monitor        Optional monitor to restrict operation's operating conditions.
     */
    public void monitoredRun(ConditionMonitor monitor);

    /**
     * Get the first drive on left.
     *
     * @return  First drive on left.
     */
    public DcMotor getFirstLeftDrive();

    /**
     * Get the first drive on right.
     *
     * @return  First drive on right.
     */
    public DcMotor getFirstRightDrive();

    /**
     * Get the last drive on left.
     *
     * @return  Last drive on left.
     */
    public DcMotor getLastLeftDrive();

    /**
     * Get the last drive on right.
     *
     * @return  Last drive on right.
     */
    public DcMotor getLastRightDrive();

    public void printCurrentStatus();
}
