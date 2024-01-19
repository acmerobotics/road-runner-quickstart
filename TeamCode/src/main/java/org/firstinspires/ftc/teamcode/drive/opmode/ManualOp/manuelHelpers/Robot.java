package org.firstinspires.ftc.teamcode.drive.opmode.ManualOp.manuelHelpers;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Hardware definitions and access for a robot with a four-motor
 * drive train and a gyro sensor.
 */

public class Robot {

    // Create a HardwareMap object to communicate with the Rev Control Hub
    private final HardwareMap hardwareMap;

    // Create a Telemetry object to communicate with the Driver Control Hub
    private final Telemetry telemetry;

    // Create motor objects
    private final DcMotor leftFront, leftRear, rightFront, rightRear;
    private final DcMotor slideLeft, slideRight, slideTop;

    // Create IMU object
    private final IMU imu;

    // Create servo objects
    private final Servo  wristGripServo, gripServo;
   // private final Servo leftGripServo, rightGripServo, wristGripServo, gripServo;

    // Create variables for headless operation
    private double headingOffset = 0.0;  // Allows headless mode to correct for rotation
    private Orientation angles;          // Uses builtin libraries to retrieve heading angle
    private Acceleration gravity;        // Builtin function for the control hub to find orientation

    // Create the Robot class constructor
    public Robot(final HardwareMap _hardwareMap, final Telemetry _telemetry) {
        // Pass variables through to Robot class to avoid conflicts
        hardwareMap = _hardwareMap;
        telemetry = _telemetry;

        // Ask the Driver Hub which port each motor is attached to based on robot config
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");

        // Reverse left side motors
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set up gyro

        // Ask the Driver Hub for our IMU
        imu = hardwareMap.get(IMU.class, "imu");
        // Create a parameters object so we can customize our IMU setup
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );

        /*
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Set our units
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Tell our IMU where to look for its calibration
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        // Set up logging
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        // Tell the imu that we don't need it to integrate velocity and position from acceleration
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        // Initialize our IMU using the parameters we just set
        */


        // Ask the Driver Hub which port each slide motor is attached to based on robot config
        slideLeft = hardwareMap.dcMotor.get("slideLeft");
        slideRight = hardwareMap.dcMotor.get("slideRight");
        slideTop = hardwareMap.dcMotor.get("intakeMotor");

        // Reverse one slide motor. This must be done since we are using a mirrored Viper slide and
        // the motors will fight if one is not reversed
        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Tell the slide motors to brake whenever we don't give any input
        // This helps hold the slide still and reduce the workload on the arm driver
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Ask the Driver Hub which port each servo is attached to based on robot config
     //   leftGripServo = hardwareMap.servo.get("leftGripServo");
      //  rightGripServo = hardwareMap.servo.get("rightGripServo");
        gripServo = hardwareMap.servo.get("gripServo");
        wristGripServo = hardwareMap.servo.get("wristGripServo");
    }

    // This function is used to update the RunMode of multiples DcMotors at once
    // It takes a RunMode argument followed by a list of DcMotor objects
    private void setMotorMode(DcMotor.RunMode mode, DcMotor... motors) {
        // Iterate over each DcMotor object
        for (DcMotor motor : motors) {
            motor.setMode(mode);    // Set the motor mode of the current DcMotor object
        }
    }

    // Invoke setMotorMode() to turn on encoders
    public void runUsingEncoders() {
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER, leftFront, leftRear, rightFront, rightRear);
    }

    // Invoke setMotorMode() to turn off encoders
    public void runWithoutEncoders() {
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, leftFront, leftRear, rightFront, rightRear);
    }

    // Invoke setMotorMode() to turn on slide encoders
    public void runSlideUsingEncoders() {
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER, slideLeft, slideRight);
    }

    // Invoke setMotorMode() to turn off slide encoders
    public void runSlideWithoutEncoders() {
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, slideLeft, slideRight);
    }

    // This function is used to update the ZeroPowerBehavior of multiples DcMotors at once
    // It takes a ZeroPowerBehavior argument followed by a list of DcMotor objects
    private void setBrake(DcMotor.ZeroPowerBehavior mode, DcMotor... motors) {
        // Iterate over each DcMotor object
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(mode);   // Set the zero power mode of the current DcMotor
        }
    }

    // Invoke setBrake() to turn on drive motor brakes
    public void runWithBrakes() {
        setBrake(DcMotor.ZeroPowerBehavior.BRAKE, leftFront, leftRear, rightFront, rightRear);
    }

    // Invoke setBrake() to turn off drive motor brakes
    public void runWithoutBrakes() {
        setBrake(DcMotor.ZeroPowerBehavior.FLOAT, leftFront, leftRear, rightFront, rightRear);
    }

    // Public Boolean wrapper to allow outside functions to access gyro calibration state
    public boolean isGyroCalibrated() {
        return true;
    }

    // This function refreshes gyro values when called by an opmode
    // These are computationally expensive tasks so don't call this function extra times for fun

    public void loop() {
        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        //    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.RADIANS);
        //   gravity
        // gravity = imu.getGravity();
    }

    // This function returns the raw heading angle of the IMU
    private double getRawHeading() {
        return angles.firstAngle;
    }

    /*
        This functions gives us our relative heading considering offset
        It subtracts the headingOffset from the raw heading
        To prevent heading outputs of greater than 360° we mod the output by 2pi
     */

    public double getHeading() {
        return (getRawHeading() - headingOffset) % (2.0 * Math.PI);
    }

    // Since the default output of the getHeading() function is in radians, this functions changes
    // it to degrees
    public double getHeadingDegrees() { return Math.toDegrees(getHeading()); }

    // Set the heading offset to our current heading
    // This means that any angles after this will be measured relative to the current heading
    public void resetHeading() {
        headingOffset = getRawHeading();
    }

    // Returns the greatest magnitude from a list of Doubles
    // The parameter xs takes a list of Double arguments
    private static double maxAbs(double... xs) {
        double ret = Double.MIN_VALUE;  // We set our return value to the minimum possible Double
        for (double x : xs) {           // Next we iterate over every Double in the list
            if (Math.abs(x) > ret) {    // If the absolute value of the current Double is greater
                // than our return variable...
                ret = Math.abs(x);      // ...we set our return variable equal to the current Double
            }
        }
        return ret;
    }

    /**
     * Set motor powers
     * <p>
     * All powers will be scaled by the greater of 1.0 or the largest absolute
     * value of any motor power.
     *
     * @param _leftFront Left front motor
     * @param _leftRear Left rear motor
     * @param _rightFront Right front motor
     * @param _rightRear Right rear motor
     */
    // Basically, if for whatever reason we request more than 100% speed from a motor, this function
    // automatically reduces the power of all the motors proportionally

    public void setMotors(double _leftFront, double _leftRear, double _rightFront, double _rightRear, double multiplier) {
        final double scale = maxAbs(1.0, _leftFront, _leftRear, _rightFront, _rightRear);
        leftFront.setPower((_leftFront * multiplier) / scale);
        leftRear.setPower((_leftRear * multiplier) / scale);
        rightFront.setPower((_rightFront * multiplier) / scale);
        rightRear.setPower((_rightRear * multiplier) / scale);
    }

    // Provide a convenient way to set all the slide motors from a single function
    public void setSlideMotors(double _slideLeft, double _slideRight, double _slideTop) {
        slideLeft.setPower(_slideLeft);
        slideRight.setPower(_slideRight);
        slideTop.setPower(_slideTop);
    }

    // This functions opens and closes our gripper based on a boolean value

    //TODO this is where we need to change the values and probably the names

    public void setGrip(boolean grip) {
        //double leftOpen = 0.0, leftClosed = 105.0;
        // double rightOpen = 270.0, rightClosed = 175.0;
        double gripOpen = 270, gripClosed = 170;

        if (grip) {
            //   leftGripServo.setPosition(leftClosed / 270);
            //   rightGripServo.setPosition(rightClosed / 270);
            gripServo.setPosition(gripOpen/270);
        } else if (!grip) {
            // leftGripServo.setPosition(leftOpen / 270);
            // rightGripServo.setPosition(rightOpen / 270);
            gripServo.setPosition(gripClosed/170);
        }
    }
    /*
    public void setGrip(boolean grip_) {
        // Hardcode our opened and closed servo positions (in degrees)
        double leftOpen = 0.0, leftClosed = 105.0;
        double rightOpen = 270.0, rightClosed = 175.0;


         /*   Dividing by 270 is required to properly set the Servo positions
            The servos require a value from 0 to 1 to set their position,
            and our particular servos have a range of 270°
            Dividing by 270 converts our degrees to a value from 0 to 1
         /
        if (grip_) {
            leftGripServo.setPosition(leftClosed / 270);
            rightGripServo.setPosition(rightClosed / 270);
        } else if (!grip_) {
            leftGripServo.setPosition(leftOpen / 270);
            rightGripServo.setPosition(rightOpen / 270);
        }
    }
    ***/
    public void setWrist (boolean wrist_) {
        double intakePos = 0.0 , backDropPos = 100;

        if (wrist_) {
            wristGripServo.setPosition(backDropPos);
        } else if (!wrist_) {
            wristGripServo.setPosition(intakePos);
        }

    }
}

