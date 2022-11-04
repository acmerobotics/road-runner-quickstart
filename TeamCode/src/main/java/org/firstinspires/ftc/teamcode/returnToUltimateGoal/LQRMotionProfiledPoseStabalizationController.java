package org.firstinspires.ftc.teamcode.returnToUltimateGoal;

import static com.ThermalEquilibrium.homeostasis.Utils.MathUtils.AngleWrap;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Math.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Utils.ExtraUtils;

public class LQRMotionProfiledPoseStabalizationController {
	private double integral_sum = 0;
	// time that the last update occured at
	protected double time_of_last_update = 0;

	// has the robot started a motion profiled move to point? this is used

	protected boolean hasStarted = false;

	// approx the number of milliseconds one loop takes
	protected double loop_time_est = 23;

	// time in milliseconds we take to accelerate
	protected double acceleration_time = 450;

	// time of starting the pose motion profiled move
	private double integral_sum_x = 0;
	private double integral_sum_y = 0;

	private double integral_sum_max = 1;
	private double integral_sum_min = -integral_sum_max;


	com.acmerobotics.roadrunner.geometry.Pose2d poseError = new com.acmerobotics.roadrunner.geometry.Pose2d(10,10,10);

	protected double startTime = 0;

	// the amount on each iteration that we increase the power scalar by inorder to ramp acceleration
	protected double rate_of_acceleration =  1 / (acceleration_time / loop_time_est);

	// the amount we scale the power by inorder to speedramp; init to rate_of_accel because its a small non-0 number.
	protected double powerScaler = rate_of_acceleration;

	protected double LQR_K = 7.234 * 1.1;
	protected double LQR_Kr = LQR_K;
	protected double LQR_scaler = 0.01;
	protected double ki = 0; //0.110;
	double kpTurn = 2.8;
	OneDimensionalLQRController translationLQRx = new OneDimensionalLQRController(LQR_K,LQR_Kr,LQR_scaler,ki);
	OneDimensionalLQRController translationLQRy = new OneDimensionalLQRController(LQR_K,LQR_Kr,LQR_scaler,ki);

	public LQRMotionProfiledPoseStabalizationController() {
	}

	public com.acmerobotics.roadrunner.geometry.Pose2d goToPosition(com.acmerobotics.roadrunner.geometry.Pose2d targetPose, com.acmerobotics.roadrunner.geometry.Pose2d robotPose) {

		if (hasStarted) {
			double time = System.currentTimeMillis();
			rate_of_acceleration = getCurrentRateOfAccel(time - time_of_last_update);
			time_of_last_update = time;
		} else {

			startTime = System.currentTimeMillis();
			time_of_last_update = startTime;

			hasStarted = true;
		}
		if (powerScaler < 1) {
			powerScaler += rate_of_acceleration;
		} else {
			powerScaler = 1;
		}


		double angle = robotPose.getHeading();
		double targetAngle = targetPose.getHeading();
		// We are epic so we assume that if the target angle is close to 180 and we are somewhat close to 180 we are at the target angle because we dont fw angle wrap
		double headingError = AngleWrap(targetAngle - angle);




		double xPower = translationLQRx.outputLQR(targetPose.getX(),robotPose.getX()) * powerScaler;
		double yPower = translationLQRy.outputLQR(targetPose.getY(),robotPose.getY()) * powerScaler;
		poseError = new com.acmerobotics.roadrunner.geometry.Pose2d(translationLQRx.getError(), translationLQRy.getError(),headingError);

		//yPower = -yPower;
		double turnPower = (headingError * kpTurn) * powerScaler;


		return new com.acmerobotics.roadrunner.geometry.Pose2d(xPower,yPower,turnPower);

	}

	/**
	 * calculates new value to ramp up by determined by the loop time
	 * @param dt delta time
	 * @return the value to increase the power level by
	 */
	protected double getCurrentRateOfAccel(double dt) {
		return 1 / (acceleration_time / dt);
	}

	public com.acmerobotics.roadrunner.geometry.Pose2d getPoseError() {
		return poseError;
	}

	public double errorMag() {
		return poseError.vec().norm();
	}

	public double headingErrorMag() {
		return Math.abs(poseError.getHeading());
	}


}
