package org.firstinspires.ftc.teamcode.azutils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;


public class IMU {
    final BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    private final IRobot robot;
    final double zeroAngle;
    final Telemetry telemetry;

    private final LinearOpMode opMode;

    public IMU(LinearOpMode opMode, IRobot robot) {

        this.opMode = opMode;
        this.robot = robot;
        telemetry = opMode.telemetry;
        BNO055IMU.Parameters IMUparameters = new BNO055IMU.Parameters();
        IMUparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMUparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUparameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        IMUparameters.loggingEnabled = true;
        IMUparameters.loggingTag = "IMU";
        IMUparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(IMUparameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        zeroAngle = absoluteAngle(getHeadingInDegrees());
    }

    public double getHeadingInDegrees() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //    return getNormalizedDegrees(angles);
        return angles.firstAngle;
    }

    public double getNormalizedDegrees(Orientation angles) {
        double degrees = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
        return AngleUnit.DEGREES.normalize(degrees);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    //@return the absolute angle of the robot at the moment from 0 to 360 degrees
    double getCorrectAngle() {

        double angle = absoluteAngle(getHeadingInDegrees());
        if (angle - zeroAngle < 0)
            return 360 - (zeroAngle - angle);
        return angle - zeroAngle;

    }

    //@return an angle between 0 and 360 degrees
    double absoluteAngle(double angle) {

        if (angle < 0) angle += 360;
        return angle;
    }

    //@return the distance in degrees between two angles
    double arcDistance(double target, double currentAngle) {
        if (target - currentAngle < 0)
            return 360 - (currentAngle - target);
        return target - currentAngle;

    }

    //turns the robot left to an absolute position
    public void turnLeftAbsolute(double target) {
        double range = 1;
        double kP = robot.getKp();
        double kI = robot.getKi();
        double kD = robot.getKd();
        //degrees = 360 - degrees;

        double angle = getCorrectAngle();

        double power;
        double integral = 0.0;
        double derivative;
        double previous_error = 0.0;
        while (!(angle > target - range && angle < target + range)) {
            angle = getCorrectAngle();
            double error = arcDistance(target, angle);
            integral += error;
            if (error == 0 || Math.abs(error) > 50) {
                integral = 0;
            }
            derivative = error - previous_error;
            previous_error = error;
            power = Math.abs(error * kP) + Math.abs(integral * kI) + Math.abs(derivative * kD);

            // if(Math.abs(speed) < 0.1) speed = 0.1;
            robot.turnLeft(power);
            printTelemetry();
        }
        robot.stop();
    }
//@param targetAngle is relative between 0 and 180 degrees

    void turnRight(double targetAngle) {
        double range = 1;
        double kP = robot.getKp();
        double kI = robot.getKi();
        double kD = robot.getKd();
        //degrees = 360 - degrees;

        double angle = getCorrectAngle();

        double power;
        double integral = 0.0;
        double derivative;
        double previous_error = 0.0;
        //zeroAngle = 0
        double currentAngle = 0;

        while (!(currentAngle > targetAngle - range && currentAngle < targetAngle + range)) {
            currentAngle = arcDistance(zeroAngle, getCorrectAngle());

            double error = targetAngle - currentAngle;

            telemetry.addData("error", error);
            telemetry.addData("currentAngle", currentAngle);
            telemetry.update();

            if (error == 0) integral = 0.0;
            if (Math.abs(error) > 40) integral = 0.0;

            derivative = error - previous_error;
            previous_error = error;

            power = Math.abs(error * kP) + Math.abs(integral * kI) + Math.abs(derivative * kD);

            //speed = Math.abs(error * kP);

            if (Math.abs(power) < 0.1) power = 0.1;
            robot.turnRight(power);

            printTelemetry();

        }
        robot.stop();

    }

    void printTelemetry() {
        telemetry.addData("zero angle", zeroAngle);
        telemetry.addData("actual heading", getHeadingInDegrees());
        telemetry.addData("heading", getCorrectAngle());
        telemetry.update();

    }
}
