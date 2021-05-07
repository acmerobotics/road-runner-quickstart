// ////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2018 ViBoTs Vines High School FTC-11341.
// ////////////////////////////////////////////////////////////////////////////
// SCM: $Id: RobotDriveBase.java 197 2019-11-17 23:03:29Z harshal $
// ////////////////////////////////////////////////////////////////////////////
package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.HashMap;
import java.util.IdentityHashMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Run drive base class to abstract a robot drive with two or more motors.
 *
 * @author Harshal Bharatia
 */
public abstract class RobotDriveBase implements RobotDrive
{
    /**
     * Owner op-mode for this robot drive.
     */
    protected OpMode  mOwner;

    protected TelemetryLogger mLogger;
    
    /**
     * Maps the drive to its user-assigned name.
     */
    protected IdentityHashMap<DcMotor, String> mMotorNameMap;

    /**
     * Constructor.
     *
     * @param owner     Owner op-mode for this drive.
     */
    protected RobotDriveBase(OpMode owner, TelemetryLogger logger)
    {
        mOwner = owner;
        mLogger = logger;
        mMotorNameMap = new IdentityHashMap<DcMotor, String>();
    }

    /**
     * Prepare the motor to be able to perform motion using motor encoder.
     * Stop the motor if currently not in a mode that supports encoder assisted
     * motion. If relative motion is expected also reset the encoder count.
     *
     * @param drive DC motor to be prepared for encoder motion.
     */
    protected void prepareNonEncoder(DcMotor drive)
    {
        if (continueNonEncoder(drive))
        {
            // Already using non-encoder mode - continue doing so.
            if (mLogger.isTraceEnabled())
            {
                mLogger.trace("Motor-" + mMotorNameMap.get(drive) + " already non-encoder");
            }
            return;
        }
        // Change in mode must cause motor to be first stopped and then mode to be reset.
        drive.setPower(0);
        drive.setMode(RunMode.RUN_WITHOUT_ENCODER);
        if (mLogger.isTraceEnabled())
        {
            mLogger.trace("Motor-" + mMotorNameMap.get(drive) + " changed to non-encoder");
        }
    }

    /**
     * Prepare the motor to be able to perform motion without using motor encoder.
     * Stop the motor if currently not in a mode that supports motion without
     * encoder.
     *
     * @param drive DC motor to be prepared without encoder motion.
     */
    protected void prepareEncoder(DcMotor drive, Boolean cumulative)
    {
        boolean resetRequired = cumulative == null || !cumulative.booleanValue();
        //boolean resetRequired = cumulative != null && cumulative.booleanValue();
        
        if (continueEncoder(drive))
        {
            // Already using encoder mode - continue doing so.
            if (resetRequired)
            {
                // Cumulative is not needed - set the encoder value to 0.
                drive.setMode(RunMode. STOP_AND_RESET_ENCODER);
            }
            if (mLogger.isTraceEnabled())
            {
                mLogger.trace("Motor-" + mMotorNameMap.get(drive) + " already encoder, reset=" + resetRequired);
            }
            return;
        }
        // Change in mode must cause motor to be first stopped and then mode to be reset.
        drive.setPower(0);
        drive.setMode(resetRequired ? RunMode.STOP_AND_RESET_ENCODER :
            RunMode.RUN_USING_ENCODER);
        if (mLogger.isTraceEnabled())
        {
            mLogger.trace("Motor-" + mMotorNameMap.get(drive) + " changed to encoder, reset=" + resetRequired);
        }
    }

    /**
     * Check if it is possible to continue motion using motor encoder.
     *
     * @param drive Motor to be checked.
     * @return True if it is possible to continue using motor with encoder.
     */
    protected boolean continueEncoder(DcMotor drive)
    {
        switch(drive.getMode())
        {
        case RUN_USING_ENCODER:
        case RUN_TO_POSITION:
        case STOP_AND_RESET_ENCODER:
        case RESET_ENCODERS:
        case RUN_USING_ENCODERS:
        {
            return(true);
        }
        default:
        {
            return(false);
        }
        }
    }

    /**
     * Check if it is possible to continue motion without using motor encoder.
     *
     * @param drive Motor to be checked.
     * @return True if it is possible to continue using motor without encoder.
     */
    protected boolean continueNonEncoder(DcMotor drive)
    {
        switch(drive.getMode())
        {
        case RUN_WITHOUT_ENCODER:
        case RUN_WITHOUT_ENCODERS:
        {
            return(true);
        }
        default:
        {
            return(false);
        }
        }
    }

    /**
     * Stop any motion for the specified motor.
     *
     * @param drive Motor to be stopped.
     */
    protected void stopMotor(DcMotor drive)
    {
        RunMode currMode = drive.getMode();
        drive.setPower(0);
        if (currMode ==  RunMode.RUN_TO_POSITION)
        {
            drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Update the power of an already moving motor.
     *
     * @param drive Motor whose power is to be updated.
     * @param newPower New power with which the motor must run henceforth.
     */
    protected void updatePower(DcMotor drive, double newPower)
    {
        drive.setPower(newPower);
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

    @Override
    public boolean waitLeftComplete(boolean preemptAnyComplete, ConditionMonitor monitor)
    {
        if ((monitor != null && !monitor.canContinue()) || !(mOwner instanceof LinearOpMode))
        {
            return(true);
        }

        boolean motorPending = true;
        while(((LinearOpMode)mOwner).opModeIsActive() &&
            (motorPending = (preemptAnyComplete ? !isAnyLeftComplete() : isLeftBusy())) &&
            monitorPermit(monitor))
        {}
        stopLeft();
        if (mLogger.isDebugEnabled())
        {
            mLogger.debug("waitLeft: complete " +
                (motorPending ? "preempted" : "done"));
        }
        return(motorPending);
    }

    @Override
    public boolean waitRightComplete(boolean preemptAnyComplete, ConditionMonitor monitor)
    {
        if ((monitor != null && !monitor.canContinue()) || !(mOwner instanceof LinearOpMode))
        {
            return(true);
        }
        boolean motorPending = true;
        while(((LinearOpMode) mOwner).opModeIsActive() &&
            (motorPending = (preemptAnyComplete ? !isAnyRightComplete() : isRightBusy())) &&
            monitorPermit(monitor))
        {}
        stopRight();
        if (mLogger.isDebugEnabled())
        {
            mLogger.debug("waitRight: completed, " +
                (motorPending ? "preempted" : "done"));
        }
        return(motorPending);
    }

    @Override
    public boolean waitAllComplete(boolean preemptAnyComplete, ConditionMonitor monitor)
    {
        if ((monitor != null && !monitor.canContinue()) || !(mOwner instanceof LinearOpMode))
        {
            return(true);
        }
        boolean motorPending = true;
        while(((LinearOpMode) mOwner).opModeIsActive() &&
            (motorPending = (preemptAnyComplete ? !isAnyComplete() : isAnyBusy())) &&
            monitorPermit(monitor))
        {}
        stopAll();
        if (mLogger.isDebugEnabled())
        {
            mLogger.debug("waitAll: completed"  +
                (motorPending ? "preempted" : "done"));
        }
        return(motorPending);
    }

    @Override
    public void monitoredRun(ConditionMonitor monitor)
    {
        if ((monitor != null && !monitor.canContinue()) || !(mOwner instanceof LinearOpMode))
        {
            return;
        }
        boolean opActive;
        while((opActive = ((LinearOpMode)mOwner).opModeIsActive()) && monitorPermit(monitor))
        {}
        stopAll();
        if (mLogger.isDebugEnabled())
        {
            mLogger.debug(String.format("monitoredRun: completed"));
        }
        return;
    }
    
    /**
     * Get the default motor name for specified motor.
     * 
     * @param motor 
     * @return  Name for the specified motor.
     */
    public static String getMotorName(DcMotor motor)
    {
        int portNumber = motor.getPortNumber();
        String connInfo = motor.getConnectionInfo();
        String[] infoPartArr = connInfo.split(";"); 
        
        String moduleId = "Default:";
        for (int i = 0; i < infoPartArr.length; i++)
        {
            String infoPart = infoPartArr[i].trim();
            if (infoPart.startsWith("module "))
            {
                String moduleIdPart = infoPart.substring("module ".length());
                moduleId = moduleIdPart != null ? (moduleIdPart.trim() + ":") : moduleId;
            }
        }
        return(moduleId + portNumber);
    }
}
