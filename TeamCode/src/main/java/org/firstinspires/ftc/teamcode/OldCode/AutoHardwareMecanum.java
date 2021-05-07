// ////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2018 ViBoTs Vines High School FTC-11341.
// ////////////////////////////////////////////////////////////////////////////
// SCM: $Id: AutoHardwareMecanum.java 191 2019-11-15 19:35:45Z veeral $
// ////////////////////////////////////////////////////////////////////////////
package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Robot hardware that manages the controls and hardware of the robot.
 *
 * @author Harshal Bharatia
 */
public class AutoHardwareMecanum
{
    /**
     * Left-back driving motor.
     */
    public DcMotor  mLeftBackDrive;

    /**
     * Left-front driving motor.
     */
    public DcMotor  mLeftFrontDrive;

    /**
     * Right-back driving motor.
     */
    public DcMotor  mRightBackDrive;

    /**
     * Right-front driving motor.
     */
    public DcMotor  mRightFrontDrive;

    /**
     * Servo to drop marker into depot.
     */
    public CRServo     mMarkerServo;    //servo port 0

    /**
     * Servo to descend robot from the lander hook.
     */
    public CRServo     mLanderServo;    //servo port 1

    /**
     * Touch sensor in front of robot body.
     */
    public TouchSensor mFrontTouch;

    /**
     * Gyro sensor.
     */
    public BNO055IMU mImu;

    /**
     * Hardware map for this runner.
     */
    private HardwareMap         mHardwareMap;

    /**
     * Owner operation-mode which controls this hardware.
     */
    private LinearOpMode        mOwner;

    /**
     * Robot drive which abstract number of drive motors used in the robot.
     */
    private RobotDrive          mRobotDrive;

    /**
     * Gyro control that manages orientation based operations for the robot.
     */
    private GyroMotionControl   mGyroControl;

    private TelemetryLogger     mLogger;

    private DigitalChannel      mLanderSwitch;

    /* Constructor */
    public AutoHardwareMecanum(LinearOpMode owner, TelemetryLogger logger)
    {
        mHardwareMap = owner.hardwareMap;
        mOwner = owner;
        mLogger = logger;
    }

    /* Initialize standard Hardware interfaces */
    public void init()
    {
        initMotors();
        initServos();
        initSwitches();
        // Since robot hangs onto hook, it is not being initialized here.
        // Such initialization shall be performed once it is resting stable onto
        // the ground.
    }

    /**
     * Define and initialize Drive Motors
     */
    private void initMotors()
    {
        // Looking from the back of the robot, the left motors are to your
        // LHS and right motors are to your RHS.
        mLeftFrontDrive  = mHardwareMap.get(DcMotor.class, "leftFront");
        mLeftBackDrive  = mHardwareMap.get(DcMotor.class, "leftBack");

        mRightFrontDrive = mHardwareMap.get(DcMotor.class, "rightFront");
        mRightBackDrive = mHardwareMap.get(DcMotor.class, "rightBack");

        // Left motors normal direction, right motors reverse direction.
        mLeftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        mLeftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        mRightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        mRightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Initialize motors to be used with encoders.
        mLeftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mLeftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mRightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mRightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mLeftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mLeftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mRightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mRightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Setting power to 0 must brake the motor by default.
        mLeftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mLeftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mRightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mRightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Create a robot drive that can abstract the robot number of drive
        // motors.
        mRobotDrive = new RobotDriveQuadMotor(mOwner, mLogger, mLeftFrontDrive,
            mLeftBackDrive, mRightFrontDrive, mRightBackDrive);

        if (mLogger.isInfoEnabled())
        {
            mLogger.info("Initialized motors for robot drive: Ports " +
                "  LF=" + mLeftFrontDrive.getPortNumber() +
                ", LB=" + mLeftBackDrive.getPortNumber() +
                ", RF=" + mRightFrontDrive.getPortNumber() +
                ", RB=" + mRightBackDrive.getPortNumber());
        }
    }

    /**
     * Define and initialize servos.
     */
    private void initServos()
    {
        mMarkerServo  = mHardwareMap.get(CRServo.class, "markerServo");
        mLanderServo = mHardwareMap.get(CRServo.class, "landerServo");
        mMarkerServo.setPower(0);
        mLanderServo.setPower(0);

        if (mLogger.isInfoEnabled())
        {
            mLogger.info("Initialized servos: Ports " +
                "Marker=" + mMarkerServo.getPortNumber() +
                ", Lander=" + mLanderServo.getPortNumber());
        }
    }

    /**
     * Define and initialize the touch sensor.
     */
    private void initSwitches()
    {
        mFrontTouch  = mHardwareMap.get(TouchSensor.class, "frontTouch");
        mLanderSwitch = mHardwareMap.get(DigitalChannel.class, "landerSwitch");
        if (mLogger.isInfoEnabled())
        {
            mLogger.info("Initialized switches: FrontTouch=" + mFrontTouch.getConnectionInfo()
                + ", Lander=" + mLanderSwitch.getConnectionInfo()
                );
        }
    }

    /**
     * Calibrate and initialize the gyro sensor.
     */
    public void initGyro()
    {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        mImu = mHardwareMap.get(BNO055IMU.class, "imu");

        mOwner.telemetry.addData("Mode", "Calibrating gyro...");
        mOwner.telemetry.update();

        mImu.initialize(parameters);

        // make sure the imu gyro is calibrated before continuing.
        while (!mOwner.isStopRequested() && !mImu.isGyroCalibrated())
        {
            mOwner.sleep(50);
            mOwner.idle();
        }
        mOwner.telemetry.addData("Mode", "Gyro calibrated");
        mOwner.telemetry.update();
        mGyroControl = new GyroMotionControl(mImu, mRobotDrive, mOwner, mLogger);

        if (mLogger.isInfoEnabled())
        {
            mLogger.info("Initialized gyro-control.");
        }
    }

    /**
     * Get the current hardware mapping.
     *
     * @return hardware map.
     */
    public HardwareMap getHardwareMap()
    {
        return(mHardwareMap);
    }

    /**
     * Get the robot drive for this hardware.
     *
     * @return Robot drive.
     */
    public RobotDrive getRobotDrive()
    {
        return(mRobotDrive);
    }

    /**
     * Get the gyro-control for this hardware.
     *
     * @return Gyro-motion control.
     */
    public GyroMotionControl getGyroControl()
    {
        return(mGyroControl);
    }

    /**
     * Get the servo to be used to drop the marker.
     *
     * @return Marker servo.
     */
    public CRServo getMarkerServo()
    {
        return(mMarkerServo);
    }

    /**
     * Get the servo to be used to descend from lander hook.
     *
     * @return Lander servo.
     */
    public CRServo getLanderServo()
    {
        return(mLanderServo);
    }

    /**
     * Get the servo to be used to drop the marker.
     *
     * @return Marker servo.
     */
    public TouchSensor getFrontTouchSensor()
    {
        return(mFrontTouch);
    }

    /**
     * Get the lander switch to be used to detect robot has landed.
     *
     * @return  Lander switch.
     */
    public DigitalChannel getLanderSwitch()
    {
        return(mLanderSwitch);
    }
}