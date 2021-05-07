// ////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2018 ViBoTs Vines High School FTC-11341.
// ////////////////////////////////////////////////////////////////////////////
// SCM: $Id: MecanumCalibrate.java 191 2019-11-15 19:35:45Z veeral $
// ////////////////////////////////////////////////////////////////////////////
package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Objects;

/**
 * OpMode to calibrate Mecanum robot hardware controls.
 *
 * Y-Stick: Push away to move forward.
 * X-Stick: Push left to move to left.
 * Face-Button-Y: Activate gyro related ops.
 * Face-Button-X: Reset related functions.
 * Face-Button-A/B: Activates in-operation additional options.
 * D-Pad: Changes control values.
 *
 * Calibration Operations:
 *
 * Drive Forward: Push L-Stick
 * Drive Backward: Pull L-Stick
 * Sideways Left: Left L-Stick
 * Sideways Right: Right L-Stick
 * Gyro Drive Forward: Push R-Stick
 * Gyro Drive Backward: Pull R-Stick
 * Gyro Rotate Left (Counterclock wise): Left R-Stick, Pivot-Left: A, Pivot-Right: B, Spin: -
 * Gyro Rotate Right (Clockwise): Right R-Stick, Pivot-Left: A, Pivot-Right: B, Spin: -
 * Lander Elevate: Y + Push L-Stick
 * Lander Lower: Y + Pull L-Stick
 * Marker Reset: Y + Left L-Stick
 * Marker Activate: Y + Right L-Stick
 * Lander Switch: Y + Push R-Stick, A cancels
 * Front Touch Switch: Y + Pull R-Stick, A cancels.
 * Sideways Left Timed: Y + Left R-Stick
 * Sideways Right Timed: Y + Right R-Stick
 *
 * Update Calibration Data:
 *
 * Distance Increase: Dpad-Up
 * Distance Decrease: Dpad-Down
 * Motor Power Increase: Dpad-Right
 * Motor Power Decrease: Dpad-Left
 * Angle Increase: Y + Dpad-Up
 * Angle Decrease: Y + Dpad-Down
 * Duration Increase: Y + Dpad-Right
 * Duration Decrease: Y + Dpad-Left
 * Reset To Defaults: X + Y + Any Dpad- Up/Down/Left/Right
 *
 * @author Harshal Bharatia
 */
@Disabled
@Autonomous(name="Mecanum: Calibrate")
public class MecanumCalibrate extends LinearOpMode
{
    public final static float STICK_THRESHOLD = 0.5f;
    /* Declare OpMode members. */
    AutoHardwareMecanum      mRobot;

    /**
     * Gyro control.
     */
    GyroMotionControl        mGyroControl;

    /**
     * Quad robot drive.
     */
    RobotDrive               mDrive;

    /**
     * Logger to display calibration/data modes.
     */
    private TelemetryLogger  mModeLogger;

    /**
     * Logger to display input for calibration operation.
     */
    private TelemetryLogger  mInputLogger;

    /**
     * Logger to display output of calibration operation.
     */
    private TelemetryLogger  mOutputLogger;

    /**
     * Logger for robot operation details.
     */
    private TelemetryLogger  mOperationLogger;

    /**
     * TImer to monitor duration of the calibrated operations.
     */
    private ElapsedTime         mTimer;

    /**
     * Calibration distance specified as encoder position target value.
     */
    private int mEncoderTarget;

    /**
     * Motor power expressed as percentage ranging for 0 to 1.
     */
    private double mMotorPower;

    /**
     * Calibration angle specified in degrees ranging from 0 to 180.
     */
    private double mAngle;

    /**
     * Duration to run a specific calibration operation in milliseconds.
     */
    private long mDuration;

    /**
     * Cache previous update-data command to detect repeated presses.
     */
    private UpdateDataCommand mPrevDataCommand;

    /**
     * Number of detected repeated presses for a update-data command - User not releasing.
     */
    private long              mConsecutiveRepeatCommand;

    /**
     * Preempt operation due to a timeout.
     */
    private ConditionMonitorFactory.TimeoutMonitor mTimeoutMonitor;

    /**
     * Preempt operation due to limit switch press.
     */
    private ConditionMonitorFactory.InputMonitor mInputMonitor;

    @Override
    public void runOpMode()
    {
        resetToFactory();
        initLogging();

        mRobot = new AutoHardwareMecanum(this, mOperationLogger);
        mRobot.init();
        mRobot.initGyro();

        ConditionMonitorFactory condFactory = ConditionMonitorFactory.getInstance();
        mTimeoutMonitor = condFactory.newTimeoutMonitor(0L);
        mInputMonitor = condFactory.newInputMonitor(mRobot.getFrontTouchSensor());
        mTimer = new ElapsedTime();

        // Wait until we're told to go
        waitForStart();

        mDrive = mRobot.getRobotDrive();
        mGyroControl = mRobot.getGyroControl();

        // Gamepad control shall be looked only when robot-drive is at rest.

        CalibrateCommand command = new CalibrateCommand();
        UpdateDataCommand dataCommand = new UpdateDataCommand();
        mPrevDataCommand = new UpdateDataCommand();

        while(opModeIsActive())
        {
            CalibrateCommand calibrateCommand = detectCalibrationOperation(command);
            if (calibrateCommand != null)
            {
                calibrate(calibrateCommand);
            }

            UpdateDataCommand requestDataCommand = detectUpdateDataMode(dataCommand);
            if (requestDataCommand != null)
            {
                updateData(requestDataCommand);
            }
        }
    }

    /**
     * Initialize the logging subsystem.
     */
    private void initLogging()
    {
        TelemetryLogger.initLoggingSubsystem(this);
        mModeLogger = new TelemetryLogger(this, "For");
        mInputLogger = new TelemetryLogger(this, "In");
        mOutputLogger = new TelemetryLogger(this, "Out");
        mOperationLogger = new TelemetryLogger(this, "Op");
    }

    /**
     * Perform operations for calibration.
     *
     * @param command   Operation to be performed.
     */
    private void calibrate(CalibrateCommand command)
    {
        double elapsedMillis;
        int position;
        // Encoder Drive controls are without Y and sticks.
        clearView();

        ConditionMonitor driveCond = mInputMonitor;
        switch(command.getMode())
        {
        case DRIVE:
        {
            if (command.isForward())
            {
                mModeLogger.info("DRIVE FORWARD [Active]");
                String message = String.format("distance: %d, power: %.2f",
                    mEncoderTarget, mMotorPower);
                mInputLogger.info(message);

                // Y is pushed forward - move motor front.
                mDrive.resetAllEncoder();
                mTimer.reset();

                mDrive.moveAll(mMotorPower, mEncoderTarget, null);
                mDrive.waitAllComplete(true, driveCond);

                logLinear(mTimer.milliseconds(), mDrive.getFirstLeftDrive().getCurrentPosition());
                mModeLogger.info("DRIVE FORWARD [Complete]");
            }
            else
            {
                mModeLogger.info("DRIVE BACKWARDS [Active]");
                String message = String.format("distance: %d, power: %.2f",
                    mEncoderTarget, mMotorPower);
                mInputLogger.info(message);

                // Y is pushed backward - move motor backwards.
                mDrive.resetAllEncoder();
                mTimer.reset();

                mDrive.moveAll(mMotorPower, -1 * mEncoderTarget, null);
                mDrive.waitAllComplete(true, driveCond);

                logLinear(mTimer.milliseconds(), mDrive.getFirstLeftDrive().getCurrentPosition());
                mModeLogger.info("DRIVE BACKWARDS [Complete]");
            }
            break;
        }
        case DRIVE_SIDEWAY:
        {
            if (mDrive instanceof RobotSidewayDrive)
            {
                if (command.isLeft())
                {
                    mModeLogger.info("DRIVE LEFT [Active]");
                    String message = String.format("distance: %d, power: %.2f",
                        mEncoderTarget, mMotorPower);
                    mInputLogger.info(message);

                    mDrive.resetAllEncoder();
                    mTimer.reset();

                    ((RobotSidewayDrive)mDrive).sidewaysLeft(mMotorPower, mEncoderTarget);
                    mDrive.waitAllComplete(true, null);

                    logLinear(mTimer.milliseconds(), mDrive.getFirstLeftDrive().getCurrentPosition());
                    mDrive.printCurrentStatus();
                    mModeLogger.info("DRIVE LEFT [Complete]");
                }
                else
                {
                    mModeLogger.info("DRIVE RIGHT [Active]");
                    String message = String.format("distance: %d, power: %.2f",
                        mEncoderTarget, mMotorPower);
                    mInputLogger.info(message);

                    mDrive.resetAllEncoder();
                    mTimer.reset();

                    ((RobotSidewayDrive)mDrive).sidewaysRight(mMotorPower, mEncoderTarget);
                    mDrive.waitAllComplete(true, null);

                    logLinear(mTimer.milliseconds(), mDrive.getFirstLeftDrive().getCurrentPosition());
                    mDrive.printCurrentStatus();
                    mModeLogger.info("DRIVE RIGHT [Complete]");
                }
            }
            break;
        }
        case DRIVE_SIDEWAY_TIMED:
        {
            if (mDrive instanceof RobotSidewayDrive)
            {
                if (command.isLeft())
                {
                    mModeLogger.info("DRIVE LEFT TIMED [Active]");
                    String message = String.format("distance: %d, power: %.2f",
                        mEncoderTarget, mMotorPower);
                    mInputLogger.info(message);

                    mDrive.resetAllEncoder();
                    mTimer.reset();

                    ((RobotSidewayDrive)mDrive).sidewaysLeft(mMotorPower);
                    mDrive.monitoredRun(mTimeoutMonitor.clearMonitor(mDuration));

                    logLinear(mTimer.milliseconds(), mDrive.getFirstLeftDrive().getCurrentPosition());
                    mModeLogger.info("DRIVE LEFT TIMED [Complete]");
                }
                else
                {
                    mModeLogger.info("DRIVE RIGHT TIMED[Active]");
                    String message = String.format("distance: %d, power: %.2f",
                        mEncoderTarget, mMotorPower);
                    mInputLogger.info(message);

                    mDrive.resetAllEncoder();
                    mTimer.reset();

                    ((RobotSidewayDrive)mDrive).sidewaysRight(mMotorPower);
                    mDrive.monitoredRun(mTimeoutMonitor.clearMonitor(mDuration));

                    logLinear(mTimer.milliseconds(), mDrive.getFirstLeftDrive().getCurrentPosition());
                    mModeLogger.info("DRIVE RIGHT TIMED [Complete]");
                }
            }
            break;
        }
        case GYRO_LINEAR:
        {
            // Gyro controls are with Y and sticks.
            if (command.isForward())
            {
                mModeLogger.info("DIRECTED FORWARD [Active]");
                String message = String.format("distance: %d, power: %.2f",
                    mEncoderTarget, mMotorPower);
                mInputLogger.info(message);

                // Y is pushed forward - move motor front.
                mDrive.resetAllEncoder();
                mGyroControl.resetAngle();
                mTimer.reset();

                mGyroControl.moveDirected(mMotorPower, mEncoderTarget, null, driveCond);

                logLinear(mTimer.milliseconds(), mDrive.getFirstLeftDrive().getCurrentPosition());
                mModeLogger.info("DIRECTED FORWARD [Complete]");
            }
            else
            {
                mModeLogger.info("DIRECTED BACKWARDS [Active]");
                String message = String.format("distance: %d, power: %.2f",
                    mEncoderTarget, mMotorPower);
                mInputLogger.info(message);

                // Y is pushed backward - move motor backwards.
                mDrive.resetAllEncoder();
                mGyroControl.resetAngle();
                mTimer.reset();
                mGyroControl.moveDirected(mMotorPower, -1 * mEncoderTarget, null, driveCond);

                logLinear(mTimer.milliseconds(), mDrive.getFirstLeftDrive().getCurrentPosition());
                mModeLogger.info("DIRECTED BACKWARDS [Complete]");
            }
            break;
        }
        case GYRO_ROTATE:
        {
            Boolean pivot = gamepad1.a ? Boolean.TRUE : gamepad1.b ? Boolean.FALSE : null;
            if (command.isLeft())
            {
                mModeLogger.info("ROTATE LEFT [Active]");
                String message = String.format("Angle: %.2f, power: %.2f, %s", mAngle, mMotorPower,
                    (pivot == null ? "spin" : (pivot == Boolean.FALSE) ? "pivotLeft" : "pivotRight"));
                mInputLogger.info(message);

                mGyroControl.resetAngle();
                mTimer.reset();

                mGyroControl.rotate(mAngle, mMotorPower, pivot, null);

                logRotate(mTimer.milliseconds(), mGyroControl.getAngle());
                mModeLogger.info("ROTATE LEFT [Complete]");
            }
            else
            {
                mModeLogger.info("ROTATE RIGHT [Active]");
                String message = String.format("Angle: %.2f, power: %.2f, %s", mAngle, mMotorPower,
                    (pivot == null ? "spin" : (pivot == Boolean.FALSE) ? "pivotLeft" : "pivotRight"));
                mInputLogger.info(message);

                mGyroControl.resetAngle();
                mTimer.reset();

                mGyroControl.rotate(-1 * mAngle, mMotorPower, pivot, null);

                logRotate(mTimer.milliseconds(), mGyroControl.getAngle());
                mModeLogger.info("ROTATE RIGHT [Complete]");
            }
            break;
        }
        case LANDER_SERVO:
        {
            // Gyro controls are with Y and sticks.
            if (command.isForward())
            {
                mModeLogger.info("LANDER ELEVATE [Active]");
                String message = String.format("duration: %d, power: %.2f",
                    mDuration, mMotorPower);
                mInputLogger.info(message);

                CRServo servo = mRobot.getLanderServo();
                servo.setDirection(DcMotorSimple.Direction.FORWARD);
                servo.setPower(mMotorPower);
                mTimer.reset();
                while(opModeIsActive() && mTimer.milliseconds() < mDuration)
                {}
                servo.setPower(0);

                mModeLogger.info("LANDER ELEVATE [Complete]");
            }
            else
            {
                mModeLogger.info("LANDER LOWER [Active]");
                String message = String.format("duration: %d, power: %.2f",
                    mDuration, mMotorPower);
                mInputLogger.info(message);

                CRServo servo = mRobot.getLanderServo();
                servo.setDirection(DcMotorSimple.Direction.REVERSE);
                servo.setPower(mMotorPower);
                mTimer.reset();
                while(opModeIsActive() && mTimer.milliseconds() < mDuration)
                {}
                servo.setPower(0);

                mModeLogger.info("LANDER LOWER [Complete]");
            }
            break;
        }
        case MARKER_SERVO:
        {
            // Gyro controls are with Y and sticks.
            if (command.isLeft())
            {
                mModeLogger.info("MARKER RESET [Active]");
                String message = String.format("duration: %d, power: %.2f",
                    mDuration, mMotorPower);
                mInputLogger.info(message);

                CRServo servo = mRobot.getMarkerServo();
                servo.setDirection(DcMotorSimple.Direction.REVERSE);
                servo.setPower(mMotorPower);
                mTimer.reset();
                while(opModeIsActive() && mTimer.milliseconds() < mDuration)
                {}
                servo.setPower(0);

                mModeLogger.info("MARKER RESET [Complete]");
            }
            else
            {
                mModeLogger.info("MARKER ACTIVATE [Active]");
                String message = String.format("duration: %d, power: %.2f",
                    mDuration, mMotorPower);
                mInputLogger.info(message);

                CRServo servo = mRobot.getMarkerServo();
                servo.setDirection(DcMotorSimple.Direction.FORWARD);
                servo.setPower(mMotorPower);
                mTimer.reset();
                while(opModeIsActive() && mTimer.milliseconds() < mDuration)
                {}
                servo.setPower(0);

                mModeLogger.info("MARKER ACTIVATE [Complete]");
            }
            break;
        }
        case LANDER_SWITCH:
        {
            mModeLogger.info("LANDER SWITCH [Active]");
            String message = String.format("Use Button-A to stop this");
            mInputLogger.info("Use Button-A to stop this");

            DigitalChannel landerSwitch = mRobot.getLanderSwitch();
            boolean started = false;
            boolean prevState = false;
            while(!gamepad1.a)
            {
                boolean state = landerSwitch.getState();
                if (!started || prevState != state)
                {
                    mOutputLogger.info("State: " + (state ? "Active" : "Inactive"));
                }
                prevState = state;
            }
            mModeLogger.info("LANDER SWITCH [Complete]");
            break;
        }
        case FRONT_SWITCH:
        {
            mModeLogger.info("FRONT SWITCH [Active]");
            String message = String.format("Use Button-A to stop this");
            mInputLogger.info("Use Button-A to stop this");

            TouchSensor touchSensor = mRobot.getFrontTouchSensor();
            boolean started = false;
            boolean prevState = false;
            while(!gamepad1.a)
            {
                boolean state = touchSensor.isPressed();
                if (!started || prevState != state)
                {
                    mOutputLogger.info("State: " + (state ? "Active" : "Inactive"));
                }
                prevState = state;
            }
            mModeLogger.info("FRONT SWITCH [Complete]");
            break;
        }
        }
    }

    /**
     * Update calibration control data.
     *
     * @param command   Data command
     */
    private void updateData(UpdateDataCommand command)
    {
        int delta;
        clearView();
        switch(command.getMode())
        {
        case RESET_ALL:
        {
            resetToFactory();
            mModeLogger.info("RESET SETTINGS");
            String message = String.format("Distance: %d, Angle: %.2f, Power: %.2f, Duration: %d",
                mEncoderTarget, mAngle, mMotorPower, mDuration);
            mOutputLogger.info(message);
            break;
        }
        case DISTANCE:
        {
            mModeLogger.info("DISTANCE");
            mEncoderTarget += (command.isCoarse() ? 100 : 1) * (command.isIncrease() ? 1 : -1);
            mEncoderTarget = (mEncoderTarget < 10) ? 100 : mEncoderTarget;
            String message = String.format("Distance: %d", mEncoderTarget);
            mOutputLogger.info(message);
            break;
        }
        case ANGLE:
        {
            mModeLogger.info("ANGLE");
            mAngle += (command.isCoarse() ? 5 : 0.25) * (command.isIncrease() ? 1 : -1);
            mAngle = Range.clip(mAngle, -180.0, 180.0);
            String message = String.format("Angle: %.2f", mAngle);
            mOutputLogger.info(message);
            break;
        }
        case POWER:
        {
            mModeLogger.info("POWER");
            mMotorPower += (command.isCoarse() ? 0.1 : 0.01) * (command.isIncrease() ? 1 : -1);
            mMotorPower = Range.clip(mMotorPower, 0.01, 1.0);
            String message = String.format("Power: %.2f", mMotorPower);
            mOutputLogger.info(message);
            break;
        }
        case DURATION:
        {
            mModeLogger.info("DURATION");
            mDuration += (command.isCoarse() ? 5000 : 100) * (command.isIncrease() ? 1 : -1);
            mDuration = mDuration < 100 ? 100 : mDuration;
            String message = String.format("Duration: %d", mDuration);
            mOutputLogger.info(message);
            break;
        }
        }
    }

    /**
     * Detect if user is requesting a calibration operation.
     *
     * @param command   Command to be populated with the operation specifics.
     * @return  Command requested by user, if any, else null.
     */
    private CalibrateCommand detectCalibrationOperation(CalibrateCommand command)
    {
        // Y control is negated to allow pushing away to be interpreted as increasing value.
        float leftFloatUpDown = gamepad1.left_stick_y * -1;
        float leftFloatSideways = gamepad1.left_stick_x;

        float rightFloatUpDown = gamepad1.right_stick_y * -1;
        float rightFloatSideways = gamepad1.right_stick_x;

        int leftSideways = Math.abs(leftFloatSideways) < STICK_THRESHOLD ? 0 : (leftFloatSideways < 0 ? -1 : 1);
        int leftUpDown = Math.abs(leftFloatUpDown) < STICK_THRESHOLD ? 0 : (leftFloatUpDown < 0 ? -1 : 1);

        int rightSideways = Math.abs(rightFloatSideways) < STICK_THRESHOLD ? 0 : (rightFloatSideways < 0 ? -1 : 1);
        int rightUpDown = Math.abs(rightFloatUpDown) < STICK_THRESHOLD ? 0 : (rightFloatUpDown < 0 ? -1 : 1);

        boolean buttonY = gamepad1.y;
        command.reset(CalibrateMode.NONE, false);

        if (leftSideways == 0 && leftUpDown == 0 && rightSideways == 0 && rightUpDown == 0)
        {
            return(null);
        }

        // No button - drive.
        if (!buttonY)
        {
            if (leftUpDown != 0)
            {
                // Move drive. forward when pushed away from user.
                return(command.reset(CalibrateMode.DRIVE, leftUpDown > 0));
            }
            else if (leftSideways != 0)
            {
                return(command.reset(CalibrateMode.DRIVE_SIDEWAY, leftSideways > 0));
            }
            if (rightUpDown != 0)
            {
                // Move drive. forward when pushed away from user.
                return(command.reset(CalibrateMode.GYRO_LINEAR, rightUpDown > 0));
            }
            else if (rightSideways != 0)
            {
                return(command.reset(CalibrateMode.GYRO_ROTATE, rightSideways > 0));
            }
        }
        else
        {
            // A button - servo.
            if (leftUpDown != 0)
            {
                // Lander servo - push away is forward and should cause hook to elevate.
                return(command.reset(CalibrateMode.LANDER_SERVO, leftUpDown > 0));
            }
            else if (leftSideways != 0)
            {
                // Left retreats, Right activates.
                return(command.reset(CalibrateMode.MARKER_SERVO, leftSideways > 0));
            }
            else if (rightSideways != 0)
            {
                return(command.reset(CalibrateMode.DRIVE_SIDEWAY_TIMED, leftSideways > 0));
            }
            else if (rightUpDown != 0)
            {
                // Lander servo - push away is forward and should cause hook to elevate.
                return(command.reset(rightUpDown > 0 ? CalibrateMode.LANDER_SWITCH :
                    CalibrateMode.FRONT_SWITCH, true));
            }
        }
        return(null);
    }

    private UpdateDataCommand detectUpdateDataMode(UpdateDataCommand command)
    {
        UpdateDataCommand newCommand = doDetectUpdateDataMode(command);
        if (newCommand != null && newCommand.equals(mPrevDataCommand))
        {
            // Rapid repeatitions of the same command changes values too quickly. Delay it.
            sleep(mConsecutiveRepeatCommand < 2 ? 800 :
                mConsecutiveRepeatCommand < 8 ? 300 : 50);
            mConsecutiveRepeatCommand++;
        }
        else
        {
            mConsecutiveRepeatCommand = 0;
            mPrevDataCommand.reset(newCommand);
        }
        return(newCommand);
    }

    /**
     * Detect if user is requesting update to the calibration data.
     *
     * @param command   Command to be populated with the data update specifics.
     * @return  Command requested by user, if any, else null.
     */
    private UpdateDataCommand doDetectUpdateDataMode(UpdateDataCommand command)
    {
        boolean padLeft = gamepad1.dpad_left;
        boolean padRight = gamepad1.dpad_right;
        boolean padUp = gamepad1.dpad_up;
        boolean padDown = gamepad1.dpad_down;

        boolean buttonY = gamepad1.y;
        boolean buttonX = gamepad1.x;
        boolean buttonB = gamepad1.b;

        if (!padUp && !padDown && !padLeft && !padRight)
        {
            // No data-command active when d-pad is inactive.
            return(null);
        }

        if (buttonX)
        {
            // X&Y with any d-pad resets the value back to factory settings.
            return(buttonY ? command.reset(UpdateDataMode.RESET_ALL, false, false) : null);
        }

        return((!buttonY && padUp) ?
                command.reset(UpdateDataMode.DISTANCE, true, buttonB) :
            (!buttonY && padDown) ?
                command.reset(UpdateDataMode.DISTANCE, false, buttonB) :
            (buttonY && padUp) ?
                command.reset(UpdateDataMode.ANGLE, true, buttonB) :
            (buttonY && padDown) ?
                command.reset(UpdateDataMode.ANGLE, false, buttonB) :
            (!buttonY && padRight) ?
                command.reset(UpdateDataMode.POWER, true, buttonB) :
            (!buttonY && padLeft) ?
                command.reset(UpdateDataMode.POWER, false, buttonB) :
            (buttonY && padRight) ?
                command.reset(UpdateDataMode.DURATION, true, buttonB) :
            (buttonY && padLeft) ?
                command.reset(UpdateDataMode.DURATION, false, buttonB) : null);
    }

    /**
     * Log linear movement result.
     * @param elapsedTime   Time it took for completion of the operation.
     * @param position      Position of encoder after the operation.
     */
    private void logLinear(double elapsedTime, int position)
    {
        String message = String.format("moved: %d, in: %.2f ms, eps: %.2f",
            position, elapsedTime, ((position * 1000)/(elapsedTime)));
        mOutputLogger.info(message);
    }

    /**
     * Log rotation movement result.
     * @param elapsedTime   Time it took for completion of the operation.
     * @param angle         Angle after the operation.
     */
    private void logRotate(double elapsedTime, double angle)
    {
        String message = String.format("rotated: %.2f, in: %.2f ms, dps: %.2f",
            angle, elapsedTime, ((angle * 1000)/(elapsedTime)));
        mOutputLogger.info(message);
    }

    private void clearView()
    {
        mModeLogger.clearTelemetry();
        mInputLogger.clearTelemetry();
        mOutputLogger.clearTelemetry();
        mOperationLogger.clearTelemetry();
    }

    /**
     * Calibration operation modes.
     *
     * @author Harshal Bharatia
     */
    private enum CalibrateMode
    {
        /**
         * No calibration operation.
         */
        NONE,
        /**
         * Drive Front-Back.
         */
        DRIVE,
        /**
         * Drive sideways.
         */
        DRIVE_SIDEWAY,
        /**
         * Drive sideways.
         */
        DRIVE_SIDEWAY_TIMED,
        /**
         * Gyro linear.
         */
        GYRO_LINEAR,
        /**
         * Gyro rotate
         */
        GYRO_ROTATE,
        /**
         * Marker servo up-down.
         */
        MARKER_SERVO,
        /**
         * Lander servo up-down.
         */
        LANDER_SERVO,

        /**
         * Lander limit switch.
         */
        LANDER_SWITCH,

        /**
         * Front touch switch
         */
        FRONT_SWITCH
    }

    /**
     * Calibration data update modes.
     *
     * @author Harshal Bharatia
     */
    private enum UpdateDataMode
    {
        /**
         * No calibration data update.
         */
        NONE,
        /**
         * Calibration distance.
         */
        DISTANCE,
        /**
         * Motor power to be used for calibration.
         */
        POWER,
        /**
         * Calibration angle.
         */
        ANGLE,
        /**
         * Duration to run for calibration.
         */
        DURATION,

        /**
         * Revert all settings to their factory default settings.
         */
        RESET_ALL
    }

    /**
     * Calibration command details.
     *
     * @author Harshal Bharatia
     */
    private class CalibrateCommand
    {
        /**
         * Stick operation mode.
         */
        CalibrateMode mMode;
        /**
         * Forward: true, Right: true, Backward: false, Left: false.
         */
        boolean   mDirection;

        public CalibrateCommand()
        {
            mMode = CalibrateMode.NONE;
            mDirection = false;
        }

        /**
         * Get calibration operation mode.
         * @return  Calibration operation
         */
        public CalibrateMode getMode()
        {
            return mMode;
        }

        /**
         * Check if forward operation is requested.
         * @return  True if forward operation is requested.
         */
        public boolean isForward()
        {
            return(mDirection);
        }

        /**
         * Check if backward operation is requested.
         * @return  True if backward operation is requested.
         */
        public boolean isBackward()
        {
            return(!mDirection);
        }

        /**
         * Check if left operation is requested.
         * @return  True if left operation is requested.
         */
        public boolean isLeft()
        {
            return(!mDirection);
        }

        /**
         * Check if right operation is requested.
         * @return  True if right operation is requested.
         */
        public boolean isRight()
        {
            return(mDirection);
        }

        @Override
        public boolean equals(Object o)
        {
            if (this == o)
            {
                return true;
            }
            if (o == null || getClass() != o.getClass())
            {
                return false;
            }
            CalibrateCommand that = (CalibrateCommand) o;
            return mDirection == that.mDirection &&
                mMode == that.mMode;
        }

        @Override
        public int hashCode()
        {
            return Objects.hash(mMode, mDirection);
        }

        /**
         * Reset the command.
         *
         * @param mode          New calibration mode.
         * @param direction     New direction.
         * @return  This instance.
         */
        public CalibrateCommand reset(CalibrateMode mode, boolean direction)
        {
            mMode = mode;
            mDirection = direction;
            return(this);
        }
    }

    /**
     * Calibration data update command details.
     *
     * @author Harshal Bharatia
     */
    private class UpdateDataCommand
    {
        /**
         * Stick operation mode.
         */
        UpdateDataMode mMode;
        /**
         * Value is to be increased.
         */
        boolean mIncrease;

        /**
         * Update using higher unit of resolution.
         */
        boolean mCoarse;

        public UpdateDataCommand()
        {
            mMode = UpdateDataMode.NONE;
            mIncrease = false;
            mCoarse = false;
        }

        @Override
        public boolean equals(Object o)
        {
            if (this == o)
            {
                return true;
            }
            if (o == null || getClass() != o.getClass())
            {
                return false;
            }
            UpdateDataCommand that = (UpdateDataCommand)o;
            return(mIncrease == that.mIncrease &&
                mCoarse == that.mCoarse &&
                mMode == that.mMode);
        }

        @Override
        public int hashCode()
        {
            return Objects.hash(mMode, mIncrease, mCoarse);
        }

        /**
         * Get update command mode.
         * @return Update command mode.
         */
        public UpdateDataMode getMode()
        {
            return mMode;
        }

        /**
         * Check if value is to be increased.
         * @return  True if value is to be increased.
         */
        public boolean isIncrease()
        {
            return mIncrease;
        }

        /**
         * Check if value is to to be updated with coarser granularity.
         * @return  True if value is to to be updated with coarser granularity.
         */
        public boolean isCoarse()
        {
            return mCoarse;
        }

        /**
         * Reset the command.
         *
         * @param mode          New update command mode.
         * @param increase      True if value is to be increased.
         * @param coarse        True of value is to to be updated with coarser granularity.
         * @return  This instance.
         */
        public UpdateDataCommand reset(UpdateDataMode mode, boolean increase, boolean coarse)
        {
            mMode = mode;
            mIncrease = increase;
            mCoarse = coarse;
            return (this);
        }

        /**
         * Copy the state from the other command.
         * @param other Other command from which state must be copied.
         * @return  This instance.
         */
        public UpdateDataCommand reset(UpdateDataCommand other)
        {
            if (other == null)
            {
                return(this);
            }
            mMode = other.mMode;
            mIncrease = other.mIncrease;
            mCoarse = other.mCoarse;
            return (this);
        }
    }

    /**
     * Reset calibration data to the factory settings.
     */
    private void resetToFactory()
    {
        mEncoderTarget = 1000;
        mMotorPower = 0.6;
        mAngle = 90;
        mDuration = 2000L;
    }
}
