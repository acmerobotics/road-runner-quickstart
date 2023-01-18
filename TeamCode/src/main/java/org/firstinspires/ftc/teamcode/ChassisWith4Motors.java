/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


/**
 * This is NOT an opmode.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are case sensitive.
 * Motors type: GoBILDA 312 RPM Yellow Jacket.
 *
 * Need config below hardware names:
 * 1. IMU - imu
 * 2. Distance sensor - fcds
 * 3. Distance sensor - rcds
 * 4. Wheel motors - names from initial function parameters.
 * 5. color sensor (Rev Color Sensor V3) - cs
 */
public class ChassisWith4Motors {
    //private
    HardwareMap hardwareMap = null;
    private final ElapsedTime runtime = new ElapsedTime();
    final double MAX_WAIT_TIME = 8.0; // in seconds
    private final boolean debugFlag = false;

    // Motors variables
    public DcMotor FrontLeftDrive = null;
    public DcMotor FrontRightDrive = null;
    public DcMotor BackLeftDrive = null;
    public DcMotor BackRightDrive = null;

    // Driving motor variables
    final double MAX_CORRECTION_POWER = 0.12;
    final double AUTO_ROTATE_POWER = 0.9;
    final double MAX_POWER = 1.0 - MAX_CORRECTION_POWER;
    final double AUTO_MAX_POWER = 0.8;
    final double SHORT_DISTANCE_POWER = 0.5;
    final double RAMP_START_POWER = 0.4;
    final double RAMP_END_POWER = 0.25;
    final double MIN_ROTATE_POWER = 0.24;

    // Position variables for autonomous
    final double COUNTS_PER_MOTOR_REV = 537.7;   // eg: GoBILDA 312 RPM Yellow Jacket
    final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    final double COUNTS_PER_INCH_DRIVE = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415); // Back-forth driving for 1 INCH. It is 42.8
    final double COUNTS_PER_INCH_STRAFE = 45; // robot strafe 1 INCH. the value is based on test (55 is wrong?)

    // power control variables
    final double RAMP_UP_DISTANCE = 10.0; // ramp up in the first 10 inch
    final double RAMP_DOWN_DISTANCE = 10.0; // slow down in the final 10 inch
    final double SHORT_DISTANCE = 6.0; // consistent low power for short driving

    // imu
    public IMU imu = null;
    YawPitchRollAngles lastAngles;
    double globalAngle = 0.0;
    double correction = 0.0;
    double rotation = 0.0;
    PIDController pidRotate, pidDrive;
    boolean resetAngleFlag = false;
    double timeMS = 0.0;
    final int INERTIA_WAIT_TIME = 500; // in ms

    //sensors
    final double DISTANCE_SENSOR_ALIGN = 5.2;
    DistanceSensor frontCenterDS = null;
    DistanceSensor backCenterDS = null;
    private DistanceSensor frontRightDS = null;
    private DistanceSensor frontLeftDS = null;

    public ColorSensor colorSensor = null;


    /**
     * Init slider motors hardware, and set their behaviors.
     *
     * @param hardwareMap the Hardware Mappings.
     */
    public void init(HardwareMap hardwareMap,
                     String frontLeftMotorName, String frontRightMotorName,
                     String backLeftMotorName, String backRightMotorName) {
        // Save reference to Hardware map
        this.hardwareMap = hardwareMap;
        Logging.log("init driving motors.");

        FrontLeftDrive = hardwareMap.get(DcMotor.class, frontLeftMotorName);
        FrontRightDrive = hardwareMap.get(DcMotor.class, frontRightMotorName);
        BackLeftDrive = hardwareMap.get(DcMotor.class, backLeftMotorName);
        BackRightDrive = hardwareMap.get(DcMotor.class, backRightMotorName);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        FrontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        FrontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        BackLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        BackRightDrive.setDirection(DcMotor.Direction.FORWARD);

        FrontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        runWithoutEncoders(); // turn off encoder mode as default

        // IMU

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw(); // reset Yaw when start initialization
        resetAngle(); // make sure last angle is initialized.

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(.019, .0, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.04, 0, 0);

        // Set up parameters for driving in a straight line.
        pidDrive.setInputRange(-90, 90);
        pidDrive.setSetpoint(0); // be sure input range has been set before
        pidDrive.setOutputRange(0, MAX_CORRECTION_POWER);
        pidDrive.enable();

        // Distance sensors
        frontCenterDS = hardwareMap.get(DistanceSensor.class, "fcds");
        backCenterDS = hardwareMap.get(DistanceSensor.class, "bcds");

        //frontLeftDS = hardwareMap.get(DistanceSensor.class, "flds");
        //frontRightDS = hardwareMap.get(DistanceSensor.class, "frds");
        //colorSensor = hardwareMap.get(ColorSensor.class, "cs");
    }

    /**
     * Run distance with color sensor, distance sensor, and motor encoders
     * Run forward until see blue or red line
     * Turn robot to 90 degree
     * run forward until distance sensor reach cone.
     */
    public void runWithMultiSensors() {
        runUsingEncoders();
        float blue = (float)colorSensor.blue();
        float red = (float)colorSensor.red();
        float r1 = red;
        float b1 = blue;
        while ((red/r1 < 1.4) && (blue/b1 < 1.4) && (getEncoderDistance() < 20))
        {
            drivingWithPID(SHORT_DISTANCE_POWER, 0 ,SHORT_DISTANCE_POWER, false); // turn off PID to speed up while loop.
            r1 = red;
            b1 = blue;
            blue = (float)colorSensor.blue();
            red = (float)colorSensor.red();
            if (debugFlag) {
                Logging.log("Red = %.1f, Blue = %.1f", red, blue);
            }
        }
        setPowers(0);
        rotateIMUTargetAngle(0);
        runUsingEncoders();
        double fcDsValue = getFcDsValue();
        while ((fcDsValue > 4.5) && (getEncoderDistance() < 20))
        {
            drivingWithPID(SHORT_DISTANCE_POWER, 0 ,0, true);
            //Logging.log("Red = %d, Green = %d, Blue = %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
            fcDsValue = getFcDsValue();
            if (debugFlag) {
                Logging.log("Distance = %.2f", fcDsValue);
            }
        }
        setPowers(0);
    }

    /**
     * Set to run to position mode for chassis motors
     *
     */
    private void runToPositionMode() {
        resetEncoders();
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Set chassis motors with run using encoders mode
     */
    public void runWithoutEncoders() {
        resetEncoders();
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    /**
     * Reset encoders and Set chassis motors with run using encoders mode
     */
    public void runUsingEncoders() {
        resetEncoders();
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * resets encoder for motors
     */
    private void resetEncoders() {
        setPowers(0);
        BackRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        zeroPowerBrake();
    }

    /**
     * Set wheels motors target positions according to back-forward moving flag
     *
     * @param tPos: target position values for motors
     * @param isBF: flag for back-forward moving or left-right moving.
     *              Back forward(1), or left right (0)
     */
    private void setTargetPositions(int tPos, boolean isBF) {
        if (isBF) {
            FrontLeftDrive.setTargetPosition(tPos);
            FrontRightDrive.setTargetPosition(tPos);
            BackLeftDrive.setTargetPosition(tPos);
            BackRightDrive.setTargetPosition(tPos);
        } else {// move left or right, positive for right
            FrontLeftDrive.setTargetPosition(tPos);
            FrontRightDrive.setTargetPosition(-tPos);
            BackLeftDrive.setTargetPosition(-tPos);
            BackRightDrive.setTargetPosition(tPos);
        }
    }

    /**
     * Set wheels motors power
     *
     * @param p: the power value set to motors (0.0 ~ 1.0)
     */
    public void setPowers(double p) {
        p = Range.clip(p, -1, 1);
        FrontLeftDrive.setPower(p);
        FrontRightDrive.setPower(p);
        BackLeftDrive.setPower(p);
        BackRightDrive.setPower(p);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle() {
        lastAngles = imu.getRobotYawPitchRollAngles();
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

        double deltaAngle = angles.getYaw(AngleUnit.DEGREES) - lastAngles.getYaw(AngleUnit.DEGREES);

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Check the first angle of IMU orientation in x-y plane
     * @return the value of first angle from IMU orientation
     */
    public double getIMUYawAngle() {
        YawPitchRollAngles imuAngles = imu.getRobotYawPitchRollAngles();
        return imuAngles.getYaw(AngleUnit.DEGREES);
    }

    /**
     * Rotate left or right to make IMU direct to a certain degree.
     *
     * @param imuTargetAngle the target angle of imu after rotation.
     */
    public void rotateIMUTargetAngle(double imuTargetAngle) {
        double imuYawAngle = getIMUYawAngle();
        rotate(-AngleUnit.DEGREES.normalize(imuYawAngle) + imuTargetAngle);
        if (debugFlag) {
            Logging.log("imu angle before rotation: %.2f", imuYawAngle);
        }
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(double degrees) {
        resetAngle();
        runWithoutEncoders(); // make sure it is the mode of Run without encoder

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359.99) {
            degrees = Math.floorMod(360, (int) degrees);
        }

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        double tolerance = 1.0;
        double rotatePower = AUTO_ROTATE_POWER;
        pidRotate.reset();
        pidRotate.setInputRange(0, degrees * 1.2);
        pidRotate.setSetpoint(degrees); // be sure input range has been set before
        pidRotate.setOutputRange(MIN_ROTATE_POWER, AUTO_ROTATE_POWER);
        pidRotate.setTolerance(tolerance);
        pidRotate.enable();

        if (Math.abs(degrees) < tolerance) {
            return;
        }

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.
        double maxLoopDuration = runtime.milliseconds();
        do {
            int[] motorsPos = {0, 0, 0, 0};
            double[] motorsPowerCorrection = {0.0, 0.0, 0.0, 0.0};
            double[] motorPowers = {rotatePower, rotatePower, rotatePower, rotatePower};
            motorsPos[0] = FrontLeftDrive.getCurrentPosition();
            motorsPos[1] = FrontRightDrive.getCurrentPosition();
            motorsPos[2] = BackLeftDrive.getCurrentPosition();
            motorsPos[3] = BackRightDrive.getCurrentPosition();

            calculateRotatePowerCorrection(motorsPos, motorsPowerCorrection);

            rotatePower = pidRotate.performPID(getAngle()); // power will be + on left turn.

            for (int i = 0; i < 4; i++) {
                motorPowers[i] = Range.clip(Math.abs(rotatePower) + motorsPowerCorrection[i],
                        0.0, Math.abs(rotatePower) * 1.1);
                motorPowers[i] = Math.copySign(Math.min(motorPowers[i], 1.0), rotatePower);
            }
            if (debugFlag) {
                Logging.log("Positions: FL = %d, FR = %d, BL = %d, BR = %d",
                        motorsPos[0], motorsPos[1], motorsPos[2], motorsPos[3]);
                Logging.log("Powers: FL = %.2f, FR = %.2f, BL = %.2f, BR = %.2f",
                        -motorPowers[0], motorPowers[1], -motorPowers[2], motorPowers[3]);
            }
            FrontLeftDrive.setPower(-motorPowers[0]);
            FrontRightDrive.setPower(motorPowers[1]);
            BackLeftDrive.setPower(-motorPowers[2]);
            BackRightDrive.setPower(motorPowers[3]);

        } while ((!pidRotate.onAbsTarget()) && ((runtime.milliseconds() - maxLoopDuration) < 1500));

        // turn the motors off.
        rightMotorSetPower(0);
        leftMotorSetPower(0);

        rotation = getAngle();

        // reset angle tracking on new heading.
        resetAngle();

        if (debugFlag) {
            Logging.log("Required turning = %.2f, Rotated = %.2f, IMU angle = %.2f.",
                    degrees, rotation, lastAngles.getYaw(AngleUnit.DEGREES));
        }
    }

    /**
     * Set left side motors power.
     *
     * @param p the power set to front left motor and back left motor
     */
    private void leftMotorSetPower(double p) {
        p = Range.clip(p, -1, 1);
        FrontLeftDrive.setPower(p);
        BackLeftDrive.setPower(p);
    }

    /**
     * Set right side motors power.
     *
     * @param p the power set to front left right motor and back right motor
     */
    private void rightMotorSetPower(double p) {
        p = Range.clip(p, -1, 1);
        FrontRightDrive.setPower(p);
        BackRightDrive.setPower(p);
    }

    /**
     * Set front motors power.
     *
     * @param p the power set to front left right motor and front right motor
     */
    private void frontMotorSetPower(double p) {
        p = Range.clip(p, -1, 1);
        FrontRightDrive.setPower(p);
        FrontLeftDrive.setPower(p);
    }

    /**
     * Set front motors power.
     *
     * @param p the power set to back left right motor and back right motor
     */
    private void backMotorSetPower(double p) {
        p = Range.clip(p, -1, 1);
        BackRightDrive.setPower(p);
        BackLeftDrive.setPower(p);
    }

    /**
     * Set motors power and drive or strafe robot straightly with run_to_position mode by PID control.
     *
     * @param tDistance   the target distance in inch
     * @param targetSign: Input value for the target distance sign to indicate drive directions. Disable PID if it is zero.
     * @param isBF:       flag for back-forth (true) moving, or left-right moving (false)
     */
    private void setPowerWithPIDControl(double tDistance, int targetSign, boolean isBF) {
        double curTime = runtime.seconds();
        boolean speedRampOn = false;
        double drivePower = SHORT_DISTANCE_POWER;
        double tDistanceAbs = Math.abs(tDistance);

        if (tDistanceAbs > SHORT_DISTANCE) {
            speedRampOn = true;
        }
        correction = 0.0;
        setPowers(RAMP_START_POWER); // p is always positive for RUN_TO_POSITION mode.

        // in seconds
        while (robotIsBusy() || (getEncoderDistance(isBF) / tDistanceAbs < 0.5) &&
                ((runtime.seconds() - curTime) < MAX_WAIT_TIME)) {
            if (0 != targetSign) { // no pid if sign = 0;
                correction = pidDrive.performPID(getAngle());
            }

            if (speedRampOn) {
                double currDistance = getEncoderDistance(isBF);
                drivePower = MAX_POWER;
                double rampUpPower = MAX_POWER;
                double rampDownPower = MAX_POWER;
                //speed ramp up.
                if (currDistance < RAMP_UP_DISTANCE) {
                    rampUpPower = (MAX_POWER - RAMP_START_POWER) * currDistance / RAMP_UP_DISTANCE + RAMP_START_POWER;
                    rampUpPower = Range.clip(rampUpPower, RAMP_START_POWER, drivePower);
                }

                // speed ramp down
                if ((tDistanceAbs - currDistance) < RAMP_DOWN_DISTANCE) {
                    rampDownPower = (MAX_POWER / 2.5 - RAMP_END_POWER) * (tDistanceAbs - currDistance) / RAMP_DOWN_DISTANCE + RAMP_END_POWER;
                    rampDownPower = Range.clip(rampDownPower, RAMP_END_POWER, drivePower);
                }

                drivePower = Math.min(rampUpPower, rampDownPower);

                if (debugFlag) {
                    Logging.log("tDistance = %.2f, currLocation = %.2f", tDistance, currDistance);
                    Logging.log("rampUpPower = %.2f, rampDownPower = %.2f", rampUpPower, rampDownPower);
                }
            }

            if (debugFlag) {
                Logging.log("power = %.2f, correction = %.2f, global angle = %.3f, last angle = %.2f",
                        drivePower, correction, getAngle(), lastAngles.getYaw(AngleUnit.DEGREES));
            }

            if (isBF) { // left motors have same power
                leftMotorSetPower(drivePower - correction * targetSign);
                rightMotorSetPower(drivePower + correction * targetSign);
            } else { // front motors have same power
                frontMotorSetPower(drivePower - correction * targetSign);
                backMotorSetPower(drivePower + correction * targetSign);
            }
        }
        stopWithBrakeAndWithoutEncoders();
        if (debugFlag) {
            Logging.log("Drive power = %.2f, global angle = %.3f", drivePower, getAngle());
        }
    }

    /**
     * Check if robot motors are busy. Return ture if yes, false otherwise.
     */
    private boolean robotIsBusy() {
        return (FrontRightDrive.isBusy() && FrontLeftDrive.isBusy() &&
                BackLeftDrive.isBusy() && BackRightDrive.isBusy());
    }

    /**
     * Calculate the motors power adjustment during rotation to make sure each motor has same
     * position counts, in order to avoid robot center shift during turning.
     *
     * @param pos                   the input of current positions for driving motors.
     * @param motorsPowerCorrection the output of power correction.
     */
    private void calculateRotatePowerCorrection(int[] pos, double[] motorsPowerCorrection) {
        int posAve = 0;
        for (int t = 0; t < 4; t++) {
            pos[t] = Math.abs(pos[t]);
            posAve += pos[t];
        }
        posAve = posAve / 4;

        // Do not correct power at the beginning when positions are small to avoid big impact on power.
        if (posAve < 50) {
            return;
        }

        for (int i = 0; i < 4; i++) {
            // the factor 4.0 below is according to test results. It can be adjust for sensitivity.
            motorsPowerCorrection[i] = (posAve - pos[i]) * 4.0 / posAve * AUTO_ROTATE_POWER;
        }
    }

    /**
     * Set target position for every wheel motor, and set power to motors to move the robot.
     * Turn off encode mode after moving. No action if moving distance less than 0.4inch (1cm).
     *
     * @param targetDistance: Input value for the target distance in inch.
     * @param isBackForth:    flag for back-forth (true) moving, or left-right moving (false)
     */
    public void runToPosition(double targetDistance, boolean isBackForth) {
        if (Math.abs(targetDistance) < 0.4) {
            return;
        }
        double countsPerInch = isBackForth? COUNTS_PER_INCH_DRIVE : COUNTS_PER_INCH_STRAFE;
        int targetPosition = (int) (targetDistance * countsPerInch);
        int tSign = (int) Math.copySign(1, targetDistance);
        setTargetPositions(targetPosition, isBackForth);

        runToPositionMode(); // turn on encoder mode, and reset encoders

        setPowerWithPIDControl(targetDistance, tSign, isBackForth);

        if (debugFlag) {
            Logging.log("Required moving distance %.2f.", targetDistance);
            Logging.log("Target Position = %d", targetPosition);
            Logging.log("PID = %d", tSign);
            Logging.log("Current Position after moving, FL = %d, FR= %d, BL = %d, BR = %d",
                    FrontLeftDrive.getCurrentPosition(), FrontRightDrive.getCurrentPosition(),
                    BackLeftDrive.getCurrentPosition(), BackRightDrive.getCurrentPosition());
        }
    }

    /**
     * Driving robot forward to the cone stack during autonomous period
     * @param checkCsDistance The distance value from robot to cone stack to check color sensor .
     * @param reachConeDistance the front center distance sensor value when robot reaching cone stack.
     */
    public void runToConeStack(double targetDistance, double checkCsDistance, double reachConeDistance) {
        float blue0 = (float)colorSensor.blue();
        float red0 = (float)colorSensor.red();
        double fcDs = getFcDsValue();
        double flDs = getFlDsValue();
        double frDs = getFrDsValue();
        double maxPower = AUTO_MAX_POWER;
        double strafePower = 0;

        runUsingEncoders();
        double currEncoder = getEncoderDistance();
        while ((fcDs > checkCsDistance) && (flDs > checkCsDistance) && (frDs > checkCsDistance) &&
                (currEncoder < targetDistance)) {
            drivingWithPID(maxPower, 0.0, 0.0, true);
            fcDs = getFcDsValue();
            flDs = getFlDsValue();
            frDs = getFrDsValue();
            currEncoder = getEncoderDistance();
            if ((fcDs < checkCsDistance + RAMP_DOWN_DISTANCE / 3.0) ||
                    (flDs < checkCsDistance  + RAMP_DOWN_DISTANCE / 3.0) ||
                    (frDs < checkCsDistance + RAMP_DOWN_DISTANCE / 3.0)) {
                maxPower = SHORT_DISTANCE_POWER;
            }
            if (debugFlag) {
                Logging.log("robot has not approached cone, fcDs = %2f, flDs = %2f, frDs = %2f", fcDs, flDs, frDs);
            }
        }

        if ((fcDs > frDs) || (fcDs > flDs)) {
            double sign = Math.copySign(1, (frDs - flDs));
            strafePower = SHORT_DISTANCE_POWER * sign;
            if (debugFlag) {
                Logging.log("cone is to the %s of robot.", (sign > 0) ? "left" : "right");
                Logging.log("robot is on coloured tape after strafe.");
            }
        }

        // driving forward to reaching cone
        int blue = colorSensor.blue();
        int red = colorSensor.red();
        if (debugFlag) {
            Logging.log("Gray color sensor values, blue = %d, red = %d.", blue, red);
        }

        double startEn = currEncoder;
        while ((fcDs > reachConeDistance) && ((currEncoder) < targetDistance)){
            drivingWithPID(RAMP_END_POWER, 0.0, strafePower, true);
            currEncoder = getEncoderDistance();
            fcDs = getFcDsValue();
            blue = colorSensor.blue();
            red = colorSensor.red();
            if ((blue / blue0 < 1.4) && (red / red0 < 1.4) && (currEncoder - startEn > Params.CHASSIS_HALF_WIDTH)) {
                strafePower = 0;
            }
            if (debugFlag) {
                Logging.log("Gray color sensor values, blue = %d, red = %d.", blue, red);
                Logging.log("fcDs = %.2f", fcDs);
                Logging.log("encoder distance during driving  %.2f", getEncoderDistance(false));
            }
        }

        stopWithBrakeAndWithoutEncoders();
        if (debugFlag) {
            Logging.log("robot is ready for cone pick up.");
        }
    }

    /**
     * Move robot from cone stack to junction.
     * @param targetDistance the max target distance
     * @param robotInitLoc robot start location
     */
    public void moveToJunction(double targetDistance, int robotInitLoc) {
        runUsingEncoders();
        double direction = Math.copySign(1, targetDistance);
        while (getEncoderDistance() < targetDistance) {
            drivingWithPID(-AUTO_MAX_POWER * direction, 0.0, 0.0, true);
        }

        double startEn = getEncoderDistance();
        while ((getEncoderDistance() - startEn) < Params.HALF_MAT) {
            drivingWithPID(-RAMP_END_POWER * direction, robotInitLoc * RAMP_END_POWER, 0, true);
        }
        stopWithBrakeAndWithoutEncoders();
    }

    /**
     * driving distance controlled by distance sensor and motor encoders.
     * @param ds the distance sensor
     * @param targetDis the total target distance, "+" forward, "-" back when driving,
     *                  and "-" is right, and "+" is to left when strafe.
     * @param threshold driving stop when the distance sensor value less than threshold. Disable
     *                  distance sensor by setting it to zero.
     * @param rampUpOn driving power ramp up on / off
     * @param rampDownOn driving power ramp down on / off
     */
    public void drivingWithSensor(double targetDis,
                                  boolean drivingOrStrafe,
                                  DistanceSensor ds,
                                  double threshold,
                                  boolean rampUpOn,
                                  boolean rampDownOn) {
        double maxPower;
        double currPower = RAMP_START_POWER;
        double direct = Math.copySign(1, targetDis);
        double startEn;
        double currEn;
        double currDs;
        
        targetDis = Math.abs(targetDis);
        if (rampUpOn) {
            runUsingEncoders(); // reset encoders
        }

        startEn = getEncoderDistance(drivingOrStrafe);
        currEn = startEn;

        // ramp up power
        if (rampUpOn && (targetDis > RAMP_UP_DISTANCE / 2)) {
            while ((currEn - startEn) < RAMP_UP_DISTANCE / 2) {
                if (drivingOrStrafe) {
                    drivingWithPID(currPower * direct, 0.0, 0.0, true);
                }
                else {
                    drivingWithPID(0, 0.0, currPower * direct, true);

                }
                currEn = getEncoderDistance(drivingOrStrafe);
                if (currEn > RAMP_UP_DISTANCE / 4) {
                    currPower = SHORT_DISTANCE_POWER;
                }
                if (debugFlag) {
                    Logging.log("Current encoder distance %.2f, curr power %.2f", currEn, currPower);
                }
            }
        }

        if (targetDis > RAMP_UP_DISTANCE / 2 + threshold) {
            maxPower = AUTO_MAX_POWER;
        }
        else {
            maxPower = SHORT_DISTANCE_POWER;
        }
        
        // driving with max power
        currDs = ds.getDistance(DistanceUnit.INCH);
        currPower = maxPower;
        while (((currEn - startEn) < targetDis) && (currDs > threshold)) {
            if (drivingOrStrafe) {
                drivingWithPID(currPower * direct, 0.0, 0.0, true);
            }
            else {
                drivingWithPID(0, 0.0, currPower * direct, true);

            }
            currEn = getEncoderDistance(drivingOrStrafe);
            currDs = ds.getDistance(DistanceUnit.INCH);
            
            // ramp down
            if (rampDownOn && (currDs < RAMP_DOWN_DISTANCE / 2 + threshold)) {
                currPower = RAMP_END_POWER;
            }
            if (debugFlag) {
                Logging.log("Encoder current distance %.2f, startEn = %.2f, Current - startEn = %.2f",
                        currEn, startEn, currEn - startEn);
                Logging.log("En target = %.2f, Distance sensor %.2f, threshold = %.2f",
                        targetDis, currDs, threshold);
            }
        }
        
        // stop if ramp down is on
        if (rampDownOn) {
            stopWithBrakeAndWithoutEncoders();
        }
    }

    /**
     * used in autonomous to go to a certain junction, the distance the robot will have to run
     * is larger than EncoderRange.
     *
     * @param targetDistance Input value for the target distance in inch, which is totally driving target distance.
     * @param threshold Stop driving when distance sensor value less than it.
     */
    public void strafeToJunction(double targetDistance, double threshold) {
        runUsingEncoders();
        double driveDirection = Math.copySign(1, targetDistance);
        targetDistance = Math.abs(targetDistance);

        double currEnDist = 0.0;

        // controlled by distance sensor
        double fcDs = getFcDsValue();
        while ((fcDs > threshold) && (currEnDist < Math.abs(targetDistance))) {
            drivingWithPID(0, 0.0, -RAMP_START_POWER * driveDirection, true);
            fcDs = getFcDsValue();
            currEnDist = getEncoderDistance(false);
            if (debugFlag) {
                Logging.log("getFcDsValue = %.2f, encoder dist = %.2f", fcDs, currEnDist);
            }
        }
        stopWithBrakeAndWithoutEncoders();
    }

    /**
     * Robot drive, strafe and turn, moving with PID controlling.
     *
     * @param drive power from drive button, "+" is forward, and "-" is back
     * @param turn power from turn button
     * @param strafe power from strafe button, "-" is right, and "+" is to left
     * @param PIDEnabled flag to enable/disable PID correction.
     */
    public void drivingWithPID(double drive, double turn, double strafe, boolean PIDEnabled) {
        double FrontLeftPower;
        double FrontRightPower;
        double BackLeftPower;
        double BackRightPower;

        // only enable correction when the turn button is not pressed.
        if (Math.abs(turn) > Math.ulp(0)) {
            pidDrive.reset();
            timeMS = runtime.milliseconds();
            resetAngleFlag = true;
            resetAngle(); // Resets the cumulative angle tracking to zero.
        }

        // turn on PID after a duration time to avoid robot inertia after turning.
        if (((runtime.milliseconds() - timeMS) > INERTIA_WAIT_TIME) && resetAngleFlag) {
            resetAngleFlag = false;
            resetAngle(); // Resets the cumulative angle tracking to zero.
            pidDrive.enable();
        }

        // Use PID with imu input to drive in a straight line.
        if (PIDEnabled && ((Math.abs(drive) > Math.ulp(0)) || (Math.abs(strafe) > Math.ulp(0)))) {
            correction = pidDrive.performPID(getAngle());
        } else {
            correction = 0.0;
        }

        FrontLeftPower = Range.clip(drive - turn - strafe - correction, -1, 1);
        FrontRightPower = Range.clip(drive + turn + strafe + correction, -1, 1);
        BackLeftPower = Range.clip(drive - turn + strafe - correction, -1, 1);
        BackRightPower = Range.clip(drive + turn - strafe + correction, -1, 1);

        // Send calculated power to wheels
        FrontLeftDrive.setPower(FrontLeftPower);
        FrontRightDrive.setPower(FrontRightPower);
        BackLeftDrive.setPower(BackLeftPower);
        BackRightDrive.setPower(BackRightPower);

        if (debugFlag) {
            Logging.log("Motors power - FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)",
                    FrontLeftPower, FrontRightPower, BackLeftPower, BackRightPower);
        }
    }

    /**
     * Get the average power of driving motors.
     * @return the average power of 4 driving motors.
     */
    public double getAveragePower() {
        return (Math.abs(FrontLeftDrive.getPower()) + Math.abs(FrontRightDrive.getPower()) +
                Math.abs(BackLeftDrive.getPower()) + Math.abs(BackRightDrive.getPower()))/4.0;
    }

    /**
     * Get the front center distance sensor value.
     * @return the value of front center distance, in inch
     */
    public double getFcDsValue() {
        if (frontCenterDS == null) {
            return 2500;
        }
        else {
            return frontCenterDS.getDistance(DistanceUnit.INCH);
        }
    }

    /**
     * Get the front Left distance sensor value.
     * @return the value of front left distance, in inch
     */
    public double getFlDsValue() {
        if (frontLeftDS == null) {
            return 2500;
        }
        else {
            return frontLeftDS.getDistance(DistanceUnit.INCH) + DISTANCE_SENSOR_ALIGN;
        }
    }

    /**
     * Get the front right distance sensor value.
     * @return the value of front right distance, in inch
     */
    public double getFrDsValue() {
        if (frontRightDS == null) {
            return 2500;
        }
        else {
            return frontRightDS.getDistance(DistanceUnit.INCH) + DISTANCE_SENSOR_ALIGN;
        }
    }

    /**
     * Get the back center distance sensor value.
     * @return the value of back center distance sensor value, in inch
     */
    public double getBcDsValue() {
        if (backCenterDS == null) {
            return 2500;
        }
        else {
            return backCenterDS.getDistance(DistanceUnit.INCH);
        }
    }

    /**
     * check the current encoder positions of wheel motors
     * @return the average motors encoder position
     */
    public double getEncoderDistance() {
        return (Math.abs(FrontLeftDrive.getCurrentPosition()) +
                Math.abs(FrontRightDrive.getCurrentPosition()) +
                Math.abs(BackRightDrive.getCurrentPosition()) +
                Math.abs(BackLeftDrive.getCurrentPosition())) / 4.0 / COUNTS_PER_INCH_DRIVE;
    }

    /**
     * check the current encoder positions of wheel motors
     * @return the average motors encoder position
     */
    public double getEncoderDistance( boolean isBF) {
        double dis;
        double aveEncoder = (Math.abs(FrontLeftDrive.getCurrentPosition()) +
                Math.abs(FrontRightDrive.getCurrentPosition()) +
                Math.abs(BackRightDrive.getCurrentPosition()) +
                Math.abs(BackLeftDrive.getCurrentPosition())) / 4.0;

        if (isBF) {
            dis = aveEncoder / COUNTS_PER_INCH_DRIVE;
        }
        else {
            dis = aveEncoder / COUNTS_PER_INCH_STRAFE;
        }
        return dis;
    }

    private void stopWithBrakeAndWithoutEncoders() {
        zeroPowerBrake();
        setPowers(0.0);
        runWithoutEncoders();
    }

    private void zeroPowerBrake() {
        FrontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
