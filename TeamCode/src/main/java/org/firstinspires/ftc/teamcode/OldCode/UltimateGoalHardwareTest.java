// ////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2018 ViBoTs Vines High School FTC-11341.
// ////////////////////////////////////////////////////////////////////////////
// SCM: $Id: SkystoneTestHardware.java 192 2019-11-15 22:02:29Z harshal $
// ////////////////////////////////////////////////////////////////////////////
package org.firstinspires.ftc.teamcode.OldCode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;

/**
 * Robot hardware that manages the controls and hardware of the robot to test Skystone.
 *
 * @author Harshal Bharatia
 */
public class UltimateGoalHardwareTest
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
     * Intake motor.
     */
    public DcMotor mIntakeMotor;
    
    /**
     * Shooter wheel
     */
    public DcMotor mShooterMotor;
    /**
     * Wobble dropper servo
     */
    public Servo mDropperServo;
    /**
     *Shooter arm
     */
    public Servo mShooterServo;

    /**
     * Gyro sensor.
     */
    public BNO055IMU mImu;

    /**
     * Hardware map for this runner.
     */
    private HardwareMap     mHardwareMap;

    /**
     * Owner operation-mode which controls this hardware.
     */
    private OpMode          mOwner;

    /**
     * Robot drive which abstract number of drive motors used in the robot.
     */
    private RobotDrive      mRobotDrive;

    /**
     * Wobble Goal Arm of the robot.
     */
    public DcMotor         mWobbleArmMotor;

    /**
     * Upper arm of the robot.
     */
    //private RobotArm            mUpperArm; not used in Ultimate Goal

    /**
     * Gyro control that manages orientation based operations for the robot.
     */
    private GyroMotionControl   mGyroControl;

    /**
     * Servo on Wobble Arm to grip Wobble Goal.
     */
    public Servo           mWobbleArmServo;

    /**
     * Gripper alignment servo.
     */
    //public Servo              mGripAlignServo; not used in Ultimate Goal

    /**
     * Capstone Servo
     */
    //public Servo              mCapstoneServo; not used in Ultimate Goal

    /**
     * Gripper control left (hold-release) servo.
     */
    //public Servo              mGripControlLeftServo; not used in Ultimate Goal

    /**
     * Gripper control right (hold-release) servo.
     */
    //public Servo              mGripControlRightServo; not used in Ultimate Goal

    /**
     * Webcam,
     */
    public CameraName         mDetectionCamera;

    //private ArmAssembly         mArmAssembly; not used in Ultimate Goal

    /**
     * Logger.
     */
    private TelemetryLogger     mLogger;

    /**
     * Minimum angle (downwards) limit for upper arm.s
     */
    //private DigitalChannel      mUpperArmMinLimit;
    /**
     * Maximum angle (upwards) limit for upper arm.
     */
    //private DigitalChannel      mUpperArmMaxLimit;
    /**javascript:void(0)
     * Minimum angle (downwards) limit for lower arm.
     */
    //private DigitalChannel      mLowerArmMinLimit;
    /**
     * Maximum angle (upwards) limit for lower arm.
     */
    //private DigitalChannel      mLowerArmMaxLimit;

    /* Constructor */
    public UltimateGoalHardwareTest(OpMode owner, TelemetryLogger logger)
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

        // Uncomment this when switches get installed.
        //initSwitches();
    }

    public CameraName initDetectionCamera()
    {
        mDetectionCamera = mHardwareMap.get(WebcamName.class, "Webcam 1");
        return(mDetectionCamera);
    }

    /**
     * Define and initialize Drive Motors
     */
    public void initMotors()
    {
        // First prepare the chassis drive motors.

        // Looking from the back of the robot, the left motors are to your
        // LHS and right motors are to your RHS.
        mLeftFrontDrive  = mHardwareMap.get(DcMotor.class, "leftfront");
        mLeftBackDrive  = mHardwareMap.get(DcMotor.class, "leftback");

        mRightFrontDrive = mHardwareMap.get(DcMotor.class, "rightfront");
        mRightBackDrive = mHardwareMap.get(DcMotor.class, "rightback");

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

        //Intake motor
        mIntakeMotor = mHardwareMap.get(DcMotor.class, "intakeMotor");

        // Reverse b/c it's facing right
        mIntakeMotor.setDirection(DcMotor.Direction.REVERSE);

        // Initialize motors to be used with encoders.
        mIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Setting power to 0 must brake the motor by default.
        mIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        mShooterMotor = mHardwareMap.get(DcMotor.class, "shooterMotor");
        mShooterMotor.setDirection(DcMotor.Direction.REVERSE);
        mShooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Create a robot drive that can abstract the robot number of drive
        // motors.
        mRobotDrive = new RobotDriveQuadMotor(mOwner, mLogger, mLeftFrontDrive, "LF",
                mLeftBackDrive, "LB", mRightFrontDrive, "RF", mRightBackDrive, "RB");

        if (mLogger.isInfoEnabled())
        {
            mLogger.info("Initialized motors for robot drive: Ports " +
                    "  LF=" + mLeftFrontDrive.getPortNumber() +
                    ", LB=" + mLeftBackDrive.getPortNumber() +
                    ", RF=" + mRightFrontDrive.getPortNumber() +
                    ", RB=" + mRightBackDrive.getPortNumber());


            mLogger.info("Robot drive details: " +
                    "  LF=" + mLeftFrontDrive.getConnectionInfo() +
                    ", LB=" + mLeftBackDrive.getConnectionInfo() +
                    ", RF=" + mRightFrontDrive.getConnectionInfo() +
                    ", RB=" + mRightBackDrive.getConnectionInfo());

            mLogger.info("Drive Mode details: " +
                    "  LF=" + mLeftFrontDrive.getMode() +
                    ", LB=" + mLeftBackDrive.getMode() +
                    ", RF=" + mRightFrontDrive.getMode() +
                    ", RB=" + mRightBackDrive.getMode());

        }

        //Prepare the arm motors. 
        mWobbleArmMotor = mHardwareMap.get(DcMotor.class, "wobbleArm");
        //mUpperArmDrive = mHardwareMap.get(DcMotor.class, "upperarm"); //not used in Ultimate Goal

        //mUpperArmDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mWobbleArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //mUpperArmDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mWobbleArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Setting power to 0 must brake the arms by default.
        //mUpperArmDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mWobbleArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //mWobbleArm = new RobotArmHexCore("Lower", mOwner, mLogger, mWobbleArmMotor, false);
        //mUpperArm = new RobotArmHexCore("Upper", mOwner, mLogger, mUpperArmDrive, false);

        /*if (mLogger.isInfoEnabled())
        {
            mLogger.info("Initialized motors for robot arms: Ports " +
                    "  LA=" + mLowerArmDrive.getPortNumber() +
                    ", UA=" + mUpperArmDrive.getPortNumber());


            mLogger.info("Robot arm details: " +
                    "  LA=" + mLowerArmDrive.getConnectionInfo() +
                    ", UA=" + mUpperArmDrive.getConnectionInfo());

        }*/

        if (mLogger.isInfoEnabled())
        {
            mLogger.info("Initialized motor for robot arm: Port " +
                    "  WA=" + mWobbleArmMotor.getPortNumber());

            mLogger.info("Robot arm details: " +
                    "  WA=" + mWobbleArmMotor.getConnectionInfo());

        }

        if (mLogger.isInfoEnabled())
        {
            mLogger.info("Initialized motors for robot drive.");
        }
    }

    /**
     * Define and initialize servos.
     */
    public void initServos()
    {
        if (mLogger.isInfoEnabled())
        {
            mLogger.info("Initialized servos.");
        }
        mShooterServo = mHardwareMap.get(Servo.class, "shooterServo");
        mShooterServo.setDirection(Direction.FORWARD);
        mShooterServo.setPosition(0.0);

        mDropperServo = mHardwareMap.get(Servo.class, "wobbleDropper");
        mDropperServo.setDirection(Direction.FORWARD);
        mDropperServo.setPosition(0.0);
        

        mWobbleArmServo = mHardwareMap.get(Servo.class, "wobbleArmGrip");
        mWobbleArmServo.setDirection(Direction.FORWARD);
        mWobbleArmServo.setPosition(0.0);

        /*mCapstoneServo = mHardwareMap.get(Servo.class, "capstone");
        mCapstoneServo.setDirection(Direction.FORWARD);
        mCapstoneServo.setPosition(1.0);

        // Init the grip to hold.

        // Set the left servo in reverse so that position 1.0 opens grip to left and 0.0 sets grip to hold.
        mGripControlLeftServo = mHardwareMap.get(Servo.class, "gripleft");
        mGripControlLeftServo.setDirection(Direction.FORWARD);
        mGripControlLeftServo.setPosition(0.0);

        // Set the right servo as forward so that position 1.0 opens grip to right and 0.0 sets grip to hold.
        mGripControlRightServo = mHardwareMap.get(Servo.class, "gripright");
        mGripControlRightServo.setDirection(Direction.REVERSE);
        mGripControlRightServo.setPosition(0.0);*/
    }

    /**
     * Calibrate and initialize the gyro sensor.
     */
    public void initGyro()
    {
        if (!(mOwner instanceof LinearOpMode))
        {
            return;
        }
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
        while (!((LinearOpMode) mOwner).isStopRequested() && !mImu.isGyroCalibrated())
        {
            ((LinearOpMode) mOwner).sleep(50);
            ((LinearOpMode) mOwner).idle();
        }
        mOwner.telemetry.addData("Mode", "Gyro calibrated");
        mOwner.telemetry.update();
        mGyroControl = new GyroMotionControl(mImu, mRobotDrive, (LinearOpMode) mOwner, mLogger);

        if (mLogger.isInfoEnabled())
        {
            mLogger.info("Initialized gyro-control.");
        }
    }

    /*private void initSwitches()
    {
        mUpperArmMinLimit = mHardwareMap.get(DigitalChannel.class, "lsUpperMin");
        mUpperArmMaxLimit = mHardwareMap.get(DigitalChannel.class, "lsUpperMax");
        mLowerArmMinLimit = mHardwareMap.get(DigitalChannel.class, "lsLowerMin");
        mLowerArmMaxLimit = mHardwareMap.get(DigitalChannel.class, "lsLowerMax");
        if (mLogger.isInfoEnabled())
        {
            mLogger.info("Initialized switches: UpperMin=" + mUpperArmMinLimit.getConnectionInfo()
                    //+ ", UpperMax=" + mUpperArmMaxLimit.getConnectionInfo()
                    + ", LowerMin=" + mLowerArmMinLimit.getConnectionInfo()
                    + ", LowerMax=" + mLowerArmMaxLimit.getConnectionInfo()
            );
        }
    }

    public DigitalChannel getUpperArmMinLimit()
    {
        return mUpperArmMinLimit;
    }

    public DigitalChannel getUpperArmMaxLimit()
    {
        return mUpperArmMaxLimit;
    }

    public DigitalChannel getLowerArmMinLimit()
    {
        return mLowerArmMinLimit;
    }

    public DigitalChannel getLowerArmMaxLimit()
    {
        return mLowerArmMaxLimit;
    }

    /*public ArmAssembly getArmAssembly() //not used in Ultimate Goal
    {
        if (mArmAssembly != null)
        {
            return(mArmAssembly);
        }
        // pivot height: 19mm  = 0.748031496"
        // structure min: asin(82mm - 48mm/254mm) = 7.69260425°
        // structure max: asin(297mm-64mm/254mm) = 66.537784365°
        ArmLocator lowerLoc = new ArmLocator(10, 0.748031496, 10, 60);

        // pivot height: 19mm  = 0.748031496"
        // structure min: asin(348mm - 145mm/304.8mm) = -41.759894867°
        // structure max: asin(472mm-178mm/304.8mm) = 74.702065226°
        ArmLocator upperLoc = new ArmLocator(12, 0.748031496, -38, 72);

        /*
         *         | a
         *         | pu
         *        /|
         *   | r / |
         *   |  /  | pl
         *   | /  /|
         *   |/  / |
         *   |  /  | b
         *   |q/
         *   |/
         *   | o
         *   |
         *   |
         *   ===========
         * Change b as b1 to b2 and notice encoder value to change from b1 to b2 as B1_B2_Encoder.
         *
         * Link angle for b = asin(b/r), as distance of o from arm base plane is same as pl - b.
         * Encoder per degree (epd) = B1_B2_Encoder/[asin(b1/r) - asin(b2/r)]
         *
         * Upper arm: Up,   77 - 210 = 133mm, encoder = -2957, epd =
         * Upper arm: Down, 147 - 77 = 70mm, encoder = 1505, epd =
         *
         * If b is from ground, deduct the ground offset 48mm to get b relative to arm base plane. r = 304.8
         * Lower arm: Up, 182 -259 = 77mm, encoder = -2115
         *   EPD = -2115/[asin((182 - 48)/304.8) - asin((259 - 48)/304.8) = -2115/26.0804 - 43.8090 = 119.2983976
         * Lower arm: Down, 173 - 100 = 73mm, encoder: 2106,
         *   EPD = 2106/[asin((173- 48)/304.8) - asin((100 - 48)/304.8)] = 146.366
         *
         * Difference in up/down EPD is due to gear slippage esp. on downward motion, use upward epd for now.
         
        upperLoc.setEncoderPerDegree(119.2983976d);
        lowerLoc.setEncoderPerDegree(119.2983976d);

        // interArmHeight: 104mm = 4.094488189"
        // groundOffset: 48mm = 1.88976378"
        // gripOffset: 15mm = 0.590551181"
        mArmAssembly = new ArmAssembly(upperLoc, lowerLoc, 4.094488189d, 1.88976378d, 0.590551181);
        return(mArmAssembly);
    }*/

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
     * Get the lower arm of the robot.
     *
     * @return Lower arm.
     */
    /*public RobotArm getLowerArm() not used in Ultimate Goal
    {
        return(mLowerArm);
    }

    /**
     * Get the upper arm of the robot.
     *
     * @return Upper arm.
     */
    /*public RobotArm getUpperArm()
    {
        return(mUpperArm);
    }*/

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
     * Get the servo to align grip.
     *
     * @return Grip align servo.
     */
    /*public Servo getGripAlignServo() not used in Ultimate Goal
    {
        return(mGripAlignServo);
    }

    /**
     * Get the servo to control the grip's left flap.
     *
     * @return Grip left control servo.
     
    public Servo getGripControlLeftServo()
    {
        return(mGripControlLeftServo);
    }

    /**
     * Get the servo to control the grip's right flap.
     *
     * @return Grip right control servo.
    
    public Servo getGripControlRightServo()
    {
        return(mGripControlRightServo);
    }

    /**
     * Get the servo to operate left talon.
     *
     * @return Talon servo.
     
    public Servo getTalonServo()
    {
        return(mTalonServo);
    }

    /**
     * Servo which moves the capstone down
     *
     * @return capstone servo.
    
    public Servo getCapstoneServo()
    {
        return(mCapstoneServo);
    }*/
    
    public Servo getShooterServo()
    {
        return(mShooterServo);
    }
     
    public Servo getDropperServo()
    {
        return(mDropperServo);
    }

    /**
     * Get the camera to be used for detection.
     *
     * @return
     */
    public CameraName getDetectionCamera()
    {
        return mDetectionCamera;
    }

    //Convert Inches to Encoder Units
    public static int inchesToEncoderTicks(double inches)
    {
        // Ticks/Rev * Gearing * Rev/Diameter = ticks per inches
        double TICKS_PER_INCH = 28 * 5 * (75 * Math.PI * 1/0.0393701);
        return (int) (inches * TICKS_PER_INCH);
    }
    
}
