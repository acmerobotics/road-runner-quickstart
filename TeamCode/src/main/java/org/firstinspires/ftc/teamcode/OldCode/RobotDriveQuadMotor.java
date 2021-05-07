// ////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2018 ViBoTs Vines High School FTC-11341.
// ////////////////////////////////////////////////////////////////////////////
// SCM: $Id: RobotDriveQuadMotor.java 197 2019-11-17 23:03:29Z harshal $
// ////////////////////////////////////////////////////////////////////////////
package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

/**
 * Robot motor drive for robots with encoders and four motors -
 * one left-front, one left-back, one right-front  and
 * one right-back motors.
 *
 * <br/><b>Motor nomenclature:</b>
 * When one stands at back of a robot such when robot moves forward, it moves away from you,
 * the motors to your left are left motors and motors to your right are right motors. Also,
 * motors towards you are back motors and motors away from you are front motors.
 *
 * <br/><b>Sideways motion:</b>
 * The sideways motion is supported only when the robot-drive has Macenum wheels. If the robot-drive
 * does not have Macenum wheels, the {@link RobotSidewayDrive} implementation methods must NOT be used.
 *
 * @author Harshal Bharatia
 */
public class RobotDriveQuadMotor extends RobotDriveBase
    implements RobotDrive, RobotSidewayDrive
{
    /**
     * Motor at left front of the robot drive.
     */
    private DcMotor  mLeftFrontDrive;

    /**
     * Motor at left back of the robot drive.
     */
    private DcMotor  mLeftBackDrive;

    /**
     * Motor at right front of the robot drive.
     */
    private DcMotor  mRightFrontDrive;

    /**
     * Motor at right back of the robot drive.
     */
    private DcMotor  mRightBackDrive;

    /**
     * Constructor.
     *
     * @param owner             Linear Op-Mode that owns this drive.
     * @param leftFrontDrive    Left-front motor drive.
     * @param leftBackDrive     Left-back motor drive.
     * @param rightFrontDrive   Right-front motor drive.
     * @param rightBackDrive    Right-back motor drive.
     */
    public RobotDriveQuadMotor(OpMode owner, TelemetryLogger logger,
        DcMotor leftFrontDrive,
        DcMotor leftBackDrive,
        DcMotor rightFrontDrive,
        DcMotor rightBackDrive)
    {
        this(owner, logger, leftFrontDrive, getMotorName(leftFrontDrive),
            leftBackDrive, getMotorName(leftBackDrive),
            rightFrontDrive, getMotorName(rightFrontDrive),
            rightBackDrive, getMotorName(rightBackDrive));
    }
    
    /**
     * Constructor.
     *
     * @param owner             Linear Op-Mode that owns this drive.
     * @param leftFrontDrive    Left-front motor drive.
     * @param leftBackDrive     Left-back motor drive.beginIndex
     * @param rightFrontDrive   Right-front motor drive.
     * @param rightBackDrive    Right-back motor drive.
     */
    public RobotDriveQuadMotor(OpMode owner, TelemetryLogger logger,
        DcMotor leftFrontDrive, String leftFrontDriveName, 
        DcMotor leftBackDrive, String leftBackDriveName,
        DcMotor rightFrontDrive, String rightFrontDriveName,
        DcMotor rightBackDrive, String rightBackDriveName)
    {
        super(owner, logger);
        mLeftFrontDrive = leftFrontDrive;
        mLeftBackDrive = leftBackDrive;
        mRightFrontDrive = rightFrontDrive;
        mRightBackDrive = rightBackDrive;
        
        leftFrontDriveName = leftFrontDriveName != null ? leftFrontDriveName : getMotorName(mLeftFrontDrive);
        leftBackDriveName = leftBackDriveName != null ? leftBackDriveName : getMotorName(mLeftBackDrive);
        rightFrontDriveName = rightFrontDriveName != null ? rightFrontDriveName : getMotorName(mRightFrontDrive);
        rightBackDriveName = rightBackDriveName != null ? rightBackDriveName : getMotorName(mRightBackDrive);
        mMotorNameMap.put(mLeftFrontDrive, leftFrontDriveName);
        mMotorNameMap.put(mLeftBackDrive, leftBackDriveName);
        mMotorNameMap.put(mRightFrontDrive, rightFrontDriveName);
        mMotorNameMap.put(mRightBackDrive, rightBackDriveName);
    }

    @Override
    public void moveAll(double power)
    {
        prepareNonEncoder(mLeftFrontDrive);
        prepareNonEncoder(mLeftBackDrive);
        prepareNonEncoder(mRightFrontDrive);
        prepareNonEncoder(mRightBackDrive);
        
        mLeftFrontDrive.setPower(power);
        mRightFrontDrive.setPower(power);
        mLeftBackDrive.setPower(power);
        mRightBackDrive.setPower(power);

        if (mLogger.isDebugEnabled())
        {
            mLogger.debug(String.format("moveAll: power=%.2f", power));
        }
    }

    @Override
    public void moveAll(double power, int encoderTarget, Boolean cumulative)
    {
        prepareEncoder(mLeftFrontDrive, cumulative);
        prepareEncoder(mLeftBackDrive, cumulative);
        prepareEncoder(mRightFrontDrive, cumulative);
        prepareEncoder(mRightBackDrive, cumulative);

        mLeftFrontDrive.setMode(RunMode.RUN_TO_POSITION);
        mLeftBackDrive.setMode(RunMode.RUN_TO_POSITION);
        mRightFrontDrive.setMode(RunMode.RUN_TO_POSITION);
        mRightBackDrive.setMode(RunMode.RUN_TO_POSITION);


        double absPower = Math.abs(power);
        mLeftFrontDrive.setPower(absPower);
        mRightFrontDrive.setPower(absPower);
        mLeftBackDrive.setPower(absPower);
        mRightBackDrive.setPower(absPower);
        
        
        mLeftFrontDrive.setTargetPosition(encoderTarget);
        mLeftBackDrive.setTargetPosition(encoderTarget);
        mRightFrontDrive.setTargetPosition(encoderTarget);
        mRightBackDrive.setTargetPosition(encoderTarget);

        

        if (mLogger.isDebugEnabled())
        {
            mLogger.debug(String.format("moveAll: power=%.2f, distance=%d", power, encoderTarget));
        }
    }

    @Override
    public void moveLeft(double power)
    {
        prepareNonEncoder(mLeftFrontDrive);
        prepareNonEncoder(mLeftBackDrive);
        mLeftFrontDrive.setPower(power);
        mLeftBackDrive.setPower(power);

        if (mLogger.isDebugEnabled())
        {
            mLogger.debug(String.format("moveLeft: power=%.2f", power));
        }
    }

    @Override
    public void moveLeft(double power, int encoderTarget, Boolean cumulative)
    {
        prepareEncoder(mLeftFrontDrive, cumulative);
        prepareEncoder(mLeftBackDrive, cumulative);

        mLeftFrontDrive.setMode(RunMode.RUN_TO_POSITION);
        mLeftBackDrive.setMode(RunMode.RUN_TO_POSITION);

        mLeftFrontDrive.setTargetPosition(encoderTarget);
        mLeftBackDrive.setTargetPosition(encoderTarget);

        double absPower = Math.abs(power);
        mLeftFrontDrive.setPower(absPower);
        mLeftBackDrive.setPower(absPower);

        if (mLogger.isDebugEnabled())
        {
            mLogger.debug(String.format("moveLeft: power=%.2f, distance=%d", power, encoderTarget));
        }
    }

    @Override
    public void moveRight(double power)
    {
        prepareNonEncoder(mRightFrontDrive);
        prepareNonEncoder(mRightBackDrive);
        mRightFrontDrive.setPower(power);
        mRightBackDrive.setPower(power);

        if (mLogger.isDebugEnabled())
        {
            mLogger.debug(String.format("moveRight: power=%.2f", power));
        }
    }

    @Override
    public void moveRight(double power, int encoderTarget, Boolean cumulative)
    {
        prepareEncoder(mRightFrontDrive, cumulative);
        prepareEncoder(mRightBackDrive, cumulative);

        mRightFrontDrive.setMode(RunMode.RUN_TO_POSITION);
        mRightBackDrive.setMode(RunMode.RUN_TO_POSITION);

        mRightFrontDrive.setTargetPosition(encoderTarget);
        mRightBackDrive.setTargetPosition(encoderTarget);

        double absPower = Math.abs(power);
        mRightFrontDrive.setPower(absPower);
        mRightBackDrive.setPower(absPower);

        if (mLogger.isDebugEnabled())
        {
            mLogger.debug(String.format("moveRight: power=%.2f, distance=%d", power, encoderTarget));
        }
    }

    @Override
    public void prepareToRotate()
    {
        // Rotation cannot be performed while encoders are active - move to non-encoder mode.
        prepareNonEncoder(mLeftFrontDrive);
        prepareNonEncoder(mRightFrontDrive);
        prepareNonEncoder(mLeftBackDrive);
        prepareNonEncoder(mRightBackDrive);
    }

    @Override
    public void updateAllPower(double newPower)
    {
        updatePower(mLeftFrontDrive, newPower);
        updatePower(mRightFrontDrive, newPower);
        updatePower(mLeftBackDrive, newPower);
        updatePower(mRightBackDrive, newPower);
    }

    @Override
    public void updateLeftPower(double newPower)
    {
        updatePower(mLeftFrontDrive, newPower);
        updatePower(mLeftBackDrive, newPower);
    }

    @Override
    public void updateRightPower(double newPower)
    {
        updatePower(mRightFrontDrive, newPower);
        updatePower(mRightBackDrive, newPower);
    }

    @Override
    public void stopAll()
    {
        stopMotor(mLeftFrontDrive);
        stopMotor(mRightFrontDrive);
        stopMotor(mLeftBackDrive);
        stopMotor(mRightBackDrive);
    }

    @Override
    public void stopRight()
    {
        stopMotor(mRightFrontDrive);
        stopMotor(mRightBackDrive);
    }

    @Override
    public void stopLeft()
    {
        stopMotor(mLeftFrontDrive);
        stopMotor(mLeftBackDrive);
    }

    @Override
    public void resetAllEncoder()
    {
        mLeftFrontDrive.setMode(RunMode.STOP_AND_RESET_ENCODER);
        mRightFrontDrive.setMode(RunMode.STOP_AND_RESET_ENCODER);
        mLeftBackDrive.setMode(RunMode.STOP_AND_RESET_ENCODER);
        mRightBackDrive.setMode(RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void resetLeftEncoder()
    {
        mLeftFrontDrive.setMode(RunMode.STOP_AND_RESET_ENCODER);
        mLeftBackDrive.setMode(RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void resetRightEncoder()
    {
        mRightFrontDrive.setMode(RunMode.STOP_AND_RESET_ENCODER);
        mRightBackDrive.setMode(RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public boolean isAnyBusy()
    {
        return(mLeftFrontDrive.isBusy() ||
            mRightFrontDrive.isBusy() ||
            mLeftBackDrive.isBusy() ||
            mRightBackDrive.isBusy());
    }

    @Override
    public boolean isLeftBusy()
    {
        return(mLeftFrontDrive.isBusy() || mLeftBackDrive.isBusy());
    }

    @Override
    public boolean isRightBusy()
    {
        return(mRightFrontDrive.isBusy() || mRightBackDrive.isBusy());
    }

    @Override
    public boolean isAnyComplete()
    {
        return((!mLeftFrontDrive.isBusy()) ||
            (!mRightFrontDrive.isBusy()) ||
            (!mLeftBackDrive.isBusy()) ||
            (!mRightBackDrive.isBusy()));
    }

    @Override
    public boolean isAnyLeftComplete()
    {
        return((!mLeftFrontDrive.isBusy()) ||
            (!mLeftBackDrive.isBusy()));
    }

    @Override
    public boolean isAnyRightComplete()
    {
        return((!mRightFrontDrive.isBusy()) ||
            (!mRightBackDrive.isBusy()));
    }

    @Override
    public void sidewaysLeft(double power)
    {
        doSideways(power, true);
    }

    @Override
    public void sidewaysLeft(double power, int encoderTarget)
    {
        doSideways(power, encoderTarget, true);
    }

    @Override
    public void sidewaysRight(double power)
    {
        doSideways(power, false);
    }

    @Override
    public void sidewaysRight(double power, int encoderTarget)
    {
        doSideways(power, encoderTarget, false);
    }

    private void doSideways(double power, boolean isLeft)
    {
        double posPower = Math.abs(power);
        double negPower = -1 * posPower;

        prepareNonEncoder(mLeftFrontDrive);
        prepareNonEncoder(mLeftBackDrive);
        prepareNonEncoder(mRightFrontDrive);
        prepareNonEncoder(mRightBackDrive);

        mLeftFrontDrive.setPower(isLeft ?  negPower : posPower);
        mLeftBackDrive.setPower(isLeft ?   posPower : negPower);
        mRightFrontDrive.setPower(isLeft ? posPower : negPower);
        mRightBackDrive.setPower(isLeft ?  negPower : posPower);

        if (mLogger.isDebugEnabled())
        {
            mLogger.debug(String.format("sideways: direction=%s, power=%.2f",
                (isLeft ? "L" : "R"), power));
        }
    }

    private void doSideways(double power, int encoderTarget, boolean isLeft)
    {
        int posEncoderTarget = (encoderTarget < 0) ? (-1 * encoderTarget) : encoderTarget;
        int negEncoderTarget = -1 * posEncoderTarget;

        // Sideways motion is always relative - reset encoder values for all motors.
        prepareEncoder(mLeftFrontDrive, null);
        prepareEncoder(mLeftBackDrive, null);
        prepareEncoder(mRightFrontDrive, null);
        prepareEncoder(mRightBackDrive, null);

        mLeftFrontDrive.setMode(RunMode.RUN_TO_POSITION);
        mLeftBackDrive.setMode(RunMode.RUN_TO_POSITION);
        mRightFrontDrive.setMode(RunMode.RUN_TO_POSITION);
        mRightBackDrive.setMode(RunMode.RUN_TO_POSITION);

        int lf = mLeftFrontDrive.getCurrentPosition();
        int lb = mLeftBackDrive.getCurrentPosition();
        int rf = mRightFrontDrive.getCurrentPosition();
        int rb = mRightBackDrive.getCurrentPosition();

        mLeftFrontDrive.setTargetPosition(isLeft ?  negEncoderTarget : posEncoderTarget);
        mLeftBackDrive.setTargetPosition(isLeft ?   posEncoderTarget : negEncoderTarget);
        mRightFrontDrive.setTargetPosition(isLeft ? posEncoderTarget : negEncoderTarget);
        mRightBackDrive.setTargetPosition(isLeft ?  negEncoderTarget : posEncoderTarget);

        double absPower = Math.abs(power);
        mLeftFrontDrive.setPower(absPower);
        mRightFrontDrive.setPower(absPower);
        mLeftBackDrive.setPower(absPower);
        mRightBackDrive.setPower(absPower);

        int lfe = mLeftFrontDrive.getTargetPosition();
        int lbe = mLeftBackDrive.getTargetPosition();
        int rfe = mRightFrontDrive.getTargetPosition();
        int rbe = mRightBackDrive.getTargetPosition();

        if (mLogger.isDebugEnabled())
        {
            mLogger.debug(String.format("sideways: direction=%s, power=%.2f, distance=%d",
                (isLeft ? "L" : "R"), power, encoderTarget));

            mLogger.debug(String.format("sideways: Start LF=%d, LB=%d, RF=%d, RB=%d, End LF=%d, LB=%d, RF=%d, RB=%d",
                lf, lb, rf, rb, lfe, lbe, rfe, rbe));
        }
    }

    @Override
    public void printCurrentStatus()
    {
        int lfe = mLeftFrontDrive.getCurrentPosition();
        int lbe = mLeftBackDrive.getCurrentPosition();
        int rfe = mRightFrontDrive.getCurrentPosition();
        int rbe = mRightBackDrive.getCurrentPosition();

        mLogger.debug(String.format("QuadPos: LF=%d, LB=%d, RF=%d, RB=%d",
            lfe, lbe, rfe, rbe));
    }

    @Override
    public DcMotor getFirstLeftDrive()
    {
        return(mLeftFrontDrive);
    }

    @Override
    public DcMotor getFirstRightDrive()
    {
        return(mRightFrontDrive);
    }

    @Override
    public DcMotor getLastLeftDrive()
    {
        return(mLeftBackDrive);
    }

    @Override
    public DcMotor getLastRightDrive()
    {
        return(mRightBackDrive);
    }
    

}
