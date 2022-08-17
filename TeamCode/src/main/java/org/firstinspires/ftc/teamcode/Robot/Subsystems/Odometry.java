package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.ThermalEquilibrium.homeostasis.Utils.Vector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.Utils.ExtraUtils;

import static org.firstinspires.ftc.teamcode.Utils.ExtraUtils.drawRobot;

public class Odometry extends Subsystem {

	private BNO055IMU imu;
	public DcMotorEx leftEncoder;
	public DcMotorEx rightEncoder;
	private double leftPrev = 0;
	private double rightPrev = 0;
	double trackWidth = 35.70453809697589;
	double velocityX = 0;
	double velocityTheta = 0;
	double thetaZeroAngle = 0;

	Vector position = new Vector(3);

	ElapsedTime timer = new ElapsedTime();

	KalmanFilter kf;
	LowPassFilter lowPassFilter;

	double leftVelocity = 0;
	double rightVelocity = 0;
	double leftDelta = 0;
	double rightDelta = 0;
	@Override
	public void initAuto(HardwareMap hwMap) {
		imu = hwMap.get(BNO055IMU.class, "imu");
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.mode = BNO055IMU.SensorMode.NDOF;
		parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
		imu.initialize(parameters);

		if (Robot.IS_NEW_6wd) {
			leftEncoder = hwMap.get(DcMotorEx.class, "BackLeft");
			rightEncoder = hwMap.get(DcMotorEx.class, "BackRight");
		} else {
			leftEncoder = hwMap.get(DcMotorEx.class, "FrontLeft");
			rightEncoder = hwMap.get(DcMotorEx.class, "FrontRight");
		}
		kf = new KalmanFilter(0.3,1,3);
		lowPassFilter = new LowPassFilter(0.6);
	}

	@Override
	public void periodic() {
		double left = encoderTicksToInches(leftEncoder.getCurrentPosition());
		double right = encoderTicksToInches(rightEncoder.getCurrentPosition());
		leftVelocity = encoderTicksToInches(leftEncoder.getVelocity());
		rightVelocity = encoderTicksToInches(rightEncoder.getVelocity());
		velocityX = (leftVelocity + rightVelocity) / 2;

		leftDelta = left - leftPrev;
		rightDelta = right - rightPrev;
		leftPrev = left;
		rightPrev = right;
		double xDelta = (leftDelta + rightDelta) / 2;
		double yDelta = 0;
		double thetaDelta = (rightDelta - leftDelta) / (trackWidth);
		AngularVelocity imuAngularVelocity = imu.getAngularVelocity();
		double gyroVelocity;
		if (Robot.IS_NEW_6wd) {
			gyroVelocity = imuAngularVelocity.zRotationRate;
		} else {
			gyroVelocity = imuAngularVelocity.xRotationRate;
		}
		velocityTheta = gyroVelocity;
		Dashboard.packet.put("gyro velocity",gyroVelocity);
		Dashboard.packet.put("left encoder",leftEncoder.getCurrentPosition());
		Dashboard.packet.put("right encoder",rightEncoder.getCurrentPosition());
		Dashboard.packet.put("x angular",imuAngularVelocity.xRotationRate);
		Dashboard.packet.put("y angular",imuAngularVelocity.yRotationRate);
		Dashboard.packet.put("z angular",imuAngularVelocity.zRotationRate);

		timer.reset();

		double imuAngle = AngleUnit.normalizeRadians(imu.getAngularOrientation().firstAngle + thetaZeroAngle);

		Vector nu = new Vector(new double[] {
				xDelta,
				yDelta,
				thetaDelta
		});

		ExtraUtils.rotate(nu, imuAngle);

		try {
			position = position.add(nu);
		} catch (Exception e) {
			e.printStackTrace();
		}
		position.set(imuAngle,2);

		drawRobot(position, Dashboard.packet);

	}

	@Override
	public void shutdown() {

	}

	public void setEstimate(Vector estimate) {
		thetaZeroAngle = estimate.get(2);
		position = estimate;
	}

	public static double encoderTicksToInches(double ticks) {
		double WHEEL_RADIUS ;
		if (Robot.IS_NEW_6wd) {
			WHEEL_RADIUS = 3.0708661 / 2;
		} else {
			WHEEL_RADIUS = 3.77953 / 2;
		}
		double ticksPerRevolution = 28.0 * 13.7;
		return WHEEL_RADIUS * 2 * Math.PI * 1 * ticks / ticksPerRevolution;
	}

	public Vector getPosition() {
		return position;
	}

	public Pose2d getPose() {
		return new Pose2d(
				position.get(0),
				position.get(1),
				new Rotation2d(position.get(2))
		);
	}

	public Vector getVelocity() {
		return new Vector(new double[] {
				velocityX,
				0,
				velocityTheta
		});
	}

	public double getLeftVelocity() {
		return leftVelocity;
	}

	public double getRightVelocity() {
		return rightVelocity;
	}

	public double getLeftDelta() {
		return leftDelta;
	}

	public double getRightDelta() {
		return rightDelta;
	}
}
