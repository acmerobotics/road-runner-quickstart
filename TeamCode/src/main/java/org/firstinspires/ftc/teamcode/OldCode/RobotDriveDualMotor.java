// ////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2018 ViBoTs Vines High School FTC-11341.
// ////////////////////////////////////////////////////////////////////////////
// SCM: $Id: RobotDriveDualMotor.java 194 2019-11-16 08:55:04Z harshal $
// ////////////////////////////////////////////////////////////////////////////
package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Robot motor drive for robots with encoders and two motors - one left  and
 * one right motors.
 *
 * <br/><b>Motor nomenclature:</b>
 *  When one stands at back of a robot such when robot moves forward, it moves away from you,
 *  the motor to your left is left motor and motor to your right is the right motor.
 *
 * @author Harshal Bharatia
 */
public class RobotDriveDualMotor extends RobotDriveBase
        implements RobotDrive
{
    /**
     * Left motor drive.
     */
    private DcMotor  mLeftDrive;

    /**
     * Right motor drive.
     */
    private DcMotor  mRightDrive;

    /**
     * Constructor.
     *
     * @param owner        Linear Op-Mode that owns this drive.
     * @param leftDrive    Left motor drive.
     * @param rightDrive   Right motor drive.
     */
    public RobotDriveDualMotor(OpMode owner, TelemetryLogger logger,
        DcMotor leftDrive, DcMotor rightDrive)
    {
        super(owner, logger);
        mLeftDrive = leftDrive;
        mRightDrive = rightDrive;
    }

    @Override
    public void moveAll(double power)
    {
        prepareNonEncoder(mLeftDrive);
        prepareNonEncoder(mRightDrive);
        mLeftDrive.setPower(power);
        mRightDrive.setPower(power);
        if (mLogger.isDebugEnabled())
        {
            mLogger.debug(String.format("moveAll: power=%.2f", power));
        }
    }

    @Override
    public void moveAll(double power, int encoderTarget, Boolean cumulative)
    {
        prepareEncoder(mLeftDrive, cumulative);
        prepareEncoder(mRightDrive, cumulative);

        mLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        mLeftDrive.setTargetPosition(encoderTarget);
        mRightDrive.setTargetPosition(encoderTarget);

        mLeftDrive.setPower(Math.abs(power));
        mRightDrive.setPower(Math.abs(power));
        if (mLogger.isDebugEnabled())
        {
            mLogger.debug(String.format("moveAll: power=%.2f, distance=%d", power, encoderTarget));
        }
    }

    @Override
    public void moveLeft(double power)
    {
        prepareNonEncoder(mLeftDrive);
        mLeftDrive.setPower(power);

        if (mLogger.isDebugEnabled())
        {
            mLogger.debug(String.format("moveLeft: power=%.2f", power));
        }
    }

    @Override
    public void moveLeft(double power, int encoderTarget, Boolean cumulative)
    {
        prepareEncoder(mLeftDrive, cumulative);
        mLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mLeftDrive.setTargetPosition(encoderTarget);
        mLeftDrive.setPower(Math.abs(power));

        if (mLogger.isDebugEnabled())
        {
            mLogger.debug(String.format("moveLeft: power=%.2f, distance=%d", power, encoderTarget));
        }
    }

    @Override
    public void moveRight(double power)
    {
        prepareNonEncoder(mRightDrive);
        mRightDrive.setPower(power);

        if (mLogger.isDebugEnabled())
        {
            mLogger.debug(String.format("moveRight: power=%.2f", power));
        }
    }

    @Override
    public void moveRight(double power, int encoderTarget, Boolean cumulative)
    {
        prepareEncoder(mRightDrive, cumulative);
        mRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mRightDrive.setTargetPosition(encoderTarget);
        mRightDrive.setPower(Math.abs(power));

        if (mLogger.isDebugEnabled())
        {
            mLogger.debug(String.format("moveRight: power=%.2f, distance=%d", power, encoderTarget));
        }
    }

    @Override
    public void prepareToRotate()
    {
        // Rotation cannot be performed while encoders are active - move to non-encoder mode.
        prepareNonEncoder(mLeftDrive);
        prepareNonEncoder(mRightDrive);
    }

    @Override
    public void updateAllPower(double newPower)
    {
        updatePower(mLeftDrive, newPower);
        updatePower(mRightDrive, newPower);
    }

    @Override
    public void updateLeftPower(double newPower)
    {
        updatePower(mLeftDrive, newPower);
    }

    @Override
    public void updateRightPower(double newPower)
    {
        updatePower(mRightDrive, newPower);
    }

    @Override
    public void stopAll()
    {
        stopMotor(mLeftDrive);
        stopMotor(mRightDrive);
    }

    @Override
    public void stopLeft()
    {
        stopMotor(mLeftDrive);
    }

    @Override
    public void stopRight()
    {
        stopMotor(mRightDrive);
    }

    @Override
    public void resetAllEncoder()
    {
        mLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void resetLeftEncoder()
    {
        mLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void resetRightEncoder()
    {
        mRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public boolean isAnyBusy()
    {
        return(mLeftDrive.isBusy() || mRightDrive.isBusy());
    }

    @Override
    public boolean isLeftBusy()
    {
        return(mLeftDrive.isBusy());
    }

    @Override
    public boolean isRightBusy()
    {
        return(mRightDrive.isBusy());
    }

    @Override
    public boolean isAnyComplete()
    {
        return((!mLeftDrive.isBusy()) || (!mRightDrive.isBusy()));
    }

    @Override
    public boolean isAnyLeftComplete()
    {
        return(!mLeftDrive.isBusy());
    }

    @Override
    public boolean isAnyRightComplete()
    {
        return(!mRightDrive.isBusy());
    }

    @Override
    public DcMotor getFirstLeftDrive()
    {
        return(mLeftDrive);
    }

    @Override
    public DcMotor getFirstRightDrive()
    {
        return(mRightDrive);
    }

    @Override
    public DcMotor getLastLeftDrive()
    {
        return(mLeftDrive);
    }

    @Override
    public DcMotor getLastRightDrive()
    {
        return(mRightDrive);
    }

    @Override
    public void printCurrentStatus()
    {
        int lfe = mLeftDrive.getCurrentPosition();
        int rfe = mRightDrive.getCurrentPosition();

        mLogger.debug(String.format("QuadPos: LF=%d, LB=%d, RF=%d, RB=%d",
            lfe, rfe));
    }
}