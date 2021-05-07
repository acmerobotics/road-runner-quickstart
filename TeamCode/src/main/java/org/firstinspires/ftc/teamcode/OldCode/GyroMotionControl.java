// ////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2018 ViBoTs Vines High School FTC-11341.
// ////////////////////////////////////////////////////////////////////////////
// SCM: $Id: GyroMotionControl.java 202 2019-11-20 07:52:52Z harshal $
// ////////////////////////////////////////////////////////////////////////////
package org.firstinspires.ftc.teamcode.OldCode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

/**
 * Gyroscope assisted motion control.
 * This control currently supports gyro-assisted rotations and linear motion in a specific direction.
 *
 * @author Harshal Bharatia
 */
@Disabled
public class GyroMotionControl
{
    /**
     * Linear OpMode that owns this control.
     */
    private LinearOpMode mOwner;

    /**
     * REV robotics inbuilt gyro sensor.
     */
    private BNO055IMU   mImu;

    /**
     * Robot drive to be controlled using this gyro control.
     */
    private RobotDrive  mDrive;

    /**
     * Cumulative angle since last reset.
     */
    private double      mGlobalAngle;

    /**
     * Last known angle reported by the gyro sensor relative to which the current deviation
     * shall be computed for adding it to the global angle.
     */
    private Orientation mLastAngles;

    /**
     * Telemetry logger.
     */
    private TelemetryLogger mLogger;


    /**
     * Constructor.
     *
     * @param imu       Gyro sensor.
     * @param drive     Robot drive to be controlled by this gyro control.
     * @param owner     Owner op-mode that owns this control.
     */
    public GyroMotionControl(BNO055IMU imu, RobotDrive drive, LinearOpMode owner, TelemetryLogger logger)
    {
        mOwner = owner;
        mImu = imu;
        mDrive = drive;
        mLogger = logger;

        resetAngle();
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle()
    {
        mLastAngles = mImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        mGlobalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * <pre>
     *                N  <--- This is the magnetic north (towards earth's north pole).
     *                0
     *               1|-1
     *                |
     *         90 --------- -90
     *                |
     *             179|-179
     *               180
     *  </pre>
     *
     * @return Angle in degrees. Counter-clockwise i.e. towards left is positive and clockwise i.e.
     * towards right is negative.
     */
    public double getAngle()
    {
        // The Z-axis is the axis that is perpendicular to the ground on which the robot moves.
        // i.e. Z-axis points towards the sky and therefore, we obtain rotations of robot-drive
        // around this axis.
        //
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
        Orientation angles = mImu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - mLastAngles.firstAngle;
        if (deltaAngle < -180)
        {
            deltaAngle += 360;
        }
        else if (deltaAngle > 180)
        {
            deltaAngle -= 360;
        }
        mGlobalAngle += deltaAngle;

        mLastAngles = angles;

        return(mGlobalAngle);
    }

    /**
     * Move robot such that current direction is preserved.
     *
     * @param power Power with which the robot-drive must proceed.
     * @param encoderTarget  Encoder target distance at which the motion must stop. It determines
     *                       the distance and direction of motion.
     * @param cumulative     Optional. If true, the encoderTarget is not reset before start of the operation,
     *                       thereby disabling relative distance. It is useful when distance must
     *                       accumulate cumulatively across multiple moves.
     * @param monitor        Optional monitor to restrict operation's operating conditions.
     * @return True if terminated before motor reached its target.
     */
    public boolean moveDirected(double power, int encoderTarget, Boolean cumulative,
        ConditionMonitor monitor)
    {
        if (monitor != null && !monitor.canContinue())
        {
            return(true);
        }

        // Reset the encoder to ensure that a precise relative distance from current position is
        // achieved.
        if (!Boolean.TRUE.equals(cumulative))
        {
            mDrive.resetAllEncoder();
        }
        mDrive.moveAll(power, encoderTarget, cumulative);
        StringBuilder buf = mLogger.isTraceEnabled() ? new StringBuilder() : null;
        boolean motorPending = true;

        // At end of path, a significant correction towards left or right results in
        // imbalance in encoder counts. Therefore, truncate the path at first completion
        // by any motor and then stop the remaining that are still busy.
        while(mOwner.opModeIsActive() &&
            (motorPending = (!mDrive.isAnyComplete())) &&
            monitorPermit(monitor))
        {
            double powerCorrection = correctCoursePower();
            // if driving in reverse, the motor correction also needs to be reversed
            if (encoderTarget < 0)
            {
                powerCorrection *= -1.0;
            }

            if (powerCorrection != 0)
            {
                // Obtain new absolute powers for left and right motors.
                double leftPower = power + powerCorrection;
                double rightPower = power - powerCorrection;

                // Normalize these powers if either one exceeds +/- 1.0;
                double max = Math.max(Math.abs(leftPower), Math.abs(leftPower));
                if (max > 1.0)
                {
                    leftPower /= max;
                    rightPower /= max;
                }

                // Update the computed new powers to the motors.
                mDrive.updateLeftPower(leftPower);
                mDrive.updateRightPower(rightPower);
                if (buf != null)
                {
                    if (buf.length() > 40)
                    {
                        buf.delete(0, 10);
                    }
                    buf.append(String.format(", %.2f", (leftPower - power)));
                    mLogger.trace("GyroCorrect: " + buf.toString());
                }
                //mOwner.sleep(1000);
            }
        }
        if (mDrive.isAnyBusy()) // Stop any motors that are busy.
        {
            mDrive.stopAll();
        }
        if (mLogger.isDebugEnabled())
        {
            String message = String.format("moveDirected: power=%.2f distance=%d, %s",
                power, encoderTarget, motorPending ? "preempted" : "done");
            mLogger.debug(message);
        }
        return(motorPending);
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double correctCoursePower()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .01;

        angle = getAngle();

        if (angle == 0)
        {
            correction = 0;             // no adjustment.
        }
        else
        {
            // Gyro angles
            //
            //       N
            //      1|-1
            //90          -90
            //    180|-180
            //
            // Correction always applied only to the left wheel.
            //
            // +angle => Moving leftwards - to correct it, left wheel needs move power.
            // -angle => Moving rightwards - to correct it, left wheel needs less power.
            correction = angle;
        }

        correction = Range.clip(correction * gain, -1, 1);
        return (correction);
    }

    public boolean rotate(double degrees, double power, Boolean pivot, ConditionMonitor monitor)
    {
        return(rotate(degrees, power, pivot, monitor, null));
    }
    
    /**
     * Rotate left or right the number of degrees. The left rotation is counter-clockwise and
     * must be specified as positive value whereas right rotation is clockwise and must be specified
     * as negative value.
     *
     * @param degrees Degrees to turn, + is left - is right
     * @param power   Maximum power with which the motors must operate during the rotation.
     * @param pivot   If non-null, rotate by pivoting on the right wheel. If null, rotation is
     *                attempted by spinning both left and right wheels. Value Boolean.FALSE
     *                pivots on left wheels whereas value Boolean.TRUE pivots on right wheels.
     * @param monitor        Optional monitor to restrict operation's operating conditions.
     * @return True if rotation was preempted.
     */
    public boolean rotate(double degrees, double power, Boolean pivot, ConditionMonitor monitor, Boolean fullPowerOnly)
    {
        if (monitor != null && !monitor.canContinue())
        {
            return(true);
        }
        mDrive.prepareToRotate();

        if (degrees > 180)
        {
            if (rotateSegment(160, power, pivot, monitor, fullPowerOnly))
            {
                return(true);
            }
            mOwner.sleep(500);  // Let previous rotation stop inertia complete.
            return(rotateSegment(degrees - 160, power, pivot, monitor, fullPowerOnly));
        }
        boolean ret = rotateSegment(degrees, power, pivot, monitor, fullPowerOnly);
        if (mLogger.isDebugEnabled())
        {
            String message = String.format("Rotate: angle=%.2f power=%.2f",
                degrees, power);
            mLogger.debug(message);
        }
        return(ret);

        /*
        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
        */
    }

    /**
     * Perform rotation for specified segment for a possibly multi-segment rotation operation.
     *
     * @param degrees Degrees to turn, + is left - is right
     * @param power   Maximum power with which the motors must operate during the rotation.
     * @param pivot   If non-null, rotate by pivoting on the right wheel. If null, rotation is
     *                attempted by spinning both left and right wheels. Value Boolean.FALSE
     *                pivots on left wheels whereas value Boolean.TRUE pivots on right wheels.
     * @param monitor        Optional monitor to restrict operation's operating conditions.
     * @return True if rotation was preempted.
     */
    public boolean rotateSegment(double degrees, double power, Boolean pivot, ConditionMonitor monitor, Boolean fullPowerOnly)
    {
        // Does not support turning more than 180 degrees due to gyroscope restrictions.

        // Reset the gyro angle as the rotation is relative to this angle.
        resetAngle();
        double fullPowerDegrees = degrees;
        double fullPower = power;
        //double remainderPower = 0.2;
        //double remainderPower = 0.25;
        double remainderPower = 0;

        if (fullPowerOnly == null || Boolean.FALSE.equals(fullPowerOnly))
        {
            remainderPower = 0.35;
            if (power >= 0.60)
            {
                fullPowerDegrees = degrees * 0.65;
            }
            else
            {
                fullPowerDegrees = degrees * 0.75;
            }
        }
        
        // Perform the first major part of the rotation with full power.
        if (doRotate(fullPowerDegrees, fullPower, pivot, monitor))
        {
            return(true);
        }
        if (remainderPower > 0)
        {
            mOwner.sleep(250);
    
            // Perform the remainder rotation with less power. This helps reduce the effect of inertia
            // and prevents rotation from overshooting the specified degrees.
            return(doRotate(degrees, remainderPower, pivot, monitor));
        }
        return(false);
    }

    /**
     * Perform the actual drive rotation using gyro.
     *
     * @param degrees Total degrees to rotate.
     * @param power   Power with which the motors must operate during the rotation.
     * @param pivot   If non-null, rotate by pivoting on the right wheel. If null, rotation is
     *                attempted by spinning both left and right wheels. Value Boolean.FALSE
     *                pivots on left wheels whereas value Boolean.TRUE pivots on right wheels.
     * @param monitor        Optional monitor to restrict operation's operating conditions.
     * @return True if rotation was preempted.
     */
    private boolean doRotate(double degrees, double power, Boolean pivot, ConditionMonitor monitor)
    {
        double  leftPower, rightPower;

        if (degrees < 0)
        {
            // turn right by moving left wheels forward and right wheels backwards.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {
            // turn left by moving left wheels backwards and right wheels forwards.
            leftPower = -power;
            rightPower = power;
        }
        else
        {
            if (mLogger.isDebugEnabled())
            {
                String message = String.format("doRotate-nop: angle=%.2f power=%.2f",
                    degrees, power);
                mLogger.debug(message);
            }
            return(false);
        }

        // Start rotating.
        if (pivot != null)
        {
            if (Boolean.TRUE.equals(pivot))
            {
                // Pivot on left wheel.
                mDrive.moveRight(rightPower);
            }
            else
            {
                // Pivot on right wheels.
                mDrive.moveLeft(leftPower);
            }
        }
        else
        {
            // Spin rotate.
            mDrive.moveLeft(leftPower);
            mDrive.moveRight(rightPower);
        }

        double currAngle;
        if (mLogger.isDebugEnabled())
        {
            String message = String.format("doRotate: Degrees: %.4f, Power: %.4f, Angle: %.4f",
                degrees, power, getAngle());
            mLogger.debug(message);
        }

        boolean pending = true;
        // Monitor rotation until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (mOwner.opModeIsActive() &&
                (currAngle = getAngle()) == 0 &&
                monitorPermit(monitor))
            {
                traceRotateStatus(currAngle, degrees, leftPower, rightPower);
            }

            // Wait until current angle reaches the required degrees.
            while (mOwner.opModeIsActive() &&
                (pending = ((currAngle = getAngle()) > degrees)) &&
                monitorPermit(monitor))
            {
                traceRotateStatus(currAngle, degrees, leftPower, rightPower);
            }
        }
        else
        {
            // left turn: Wait until current angle reaches the required degrees.
            while (mOwner.opModeIsActive() &&
                (pending = ((currAngle = getAngle()) < degrees)) &&
                monitorPermit(monitor))
            {
                traceRotateStatus(currAngle, degrees, leftPower, rightPower);
            }
        }

        // turn the motors off.
        mDrive.stopAll();
        if (mLogger.isDebugEnabled())
        {
            String message = String.format("doRotate: angle=%.2f power=%.2f, pending=%s",
                degrees, power, !pending ? "done" : "preempted");
            mLogger.debug(message);
        }
        return(pending);
    }

    /**
     * Check if monitor, if present, permits the current operation to continue.
     *
     * @param monitor   Monitor being checked.
     * @return  True if monitor, if present,  permits the current operation to continue.
     */
    private boolean monitorPermit(ConditionMonitor monitor)
    {
        return(monitor == null || monitor.canContinue());
    }

    private void traceRotateStatus(double currAngle, double targetAngle, double leftPower, double rightPower)
    {
        if (mLogger.isTraceEnabled())
        {
            String message = String.format("GyroRotate: Angle[%.2f / %.2f], Power[L: %.2f, R: %.2f]",
                currAngle, targetAngle, leftPower, rightPower);
            mLogger.trace(message);
        }
    }
}
