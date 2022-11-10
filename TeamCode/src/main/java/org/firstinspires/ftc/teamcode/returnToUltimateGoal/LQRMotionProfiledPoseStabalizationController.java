package org.firstinspires.ftc.teamcode.returnToUltimateGoal;

import static com.ThermalEquilibrium.homeostasis.Utils.MathUtils.AngleWrap;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Math.Controllers.Coefficient.SqrtCoefficients;
import org.firstinspires.ftc.teamcode.Math.Controllers.SqrtControl;

public class LQRMotionProfiledPoseStabalizationController {
	private double integral_sum = 0;
	// time that the last update occured at
	protected double time_of_last_update = 0;

	// has the robot started a motion profiled move to point? this is used

	protected boolean hasStarted = false;

	// approx the number of milliseconds one loop takes
	protected double loop_time_est = 15;

	// time in milliseconds we take to accelerate
	protected double acceleration_time = 1800;

	BasicPID turnController = new BasicPID(new PIDCoefficients(0.55,0,0));
	AngleController turnControlWrapped = new AngleController(turnController);
	ElapsedTime timer = new ElapsedTime();

	com.acmerobotics.roadrunner.geometry.Pose2d poseError = new com.acmerobotics.roadrunner.geometry.Pose2d(10,10,10);

	protected double startTime = 0;

	// the amount on each iteration that we increase the power scalar by inorder to ramp acceleration
	protected double rate_of_acceleration =  1 / (acceleration_time / loop_time_est);

	// the amount we scale the power by inorder to speed-ramp; init to rate_of_accel because its a small non-0 number.
	protected double powerScalar = rate_of_acceleration;

	protected double LQR_K = 7.234 * 0.65;
	protected double LQR_Kr = LQR_K;
	protected double LQR_scalar = 0.01;
	protected double ki = 0.005;

	double previousErrorMag = 0;
	double errorMagDeriv = 10;
	OneDimensionalLQRController translationLQRx = new OneDimensionalLQRController(LQR_K,LQR_Kr, LQR_scalar,ki);
	OneDimensionalLQRController translationLQRy = new OneDimensionalLQRController(LQR_K,LQR_Kr, LQR_scalar,ki);
	PIDCoefficients coefficients = new PIDCoefficients(0.09,0,0.35);
	BasicPID pidX = new BasicPID(coefficients);
	BasicPID pidY = new BasicPID(coefficients);

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
		if (powerScalar < 1) {
			powerScalar += rate_of_acceleration;
		} else {
			powerScalar = 1;
		}


		double angle = robotPose.getHeading();
		double targetAngle = targetPose.getHeading();
		double headingError = AngleWrap(targetAngle - angle);




		double xPower = clipPower(pidX.calculate(targetPose.getX(),robotPose.getX()));//clipPower(translationLQRx.outputLQR(targetPose.getX(),robotPose.getX()));
		double yPower = clipPower(pidY.calculate(targetPose.getY(),robotPose.getY())); //clipPower(translationLQRy.outputLQR(targetPose.getY(),robotPose.getY()));
		Vector2d normalized = normalizePower(xPower,yPower).times(powerScalar);
		xPower = normalized.getX();
		yPower = normalized.getY();
		double errorX = targetPose.getX() - robotPose.getX();
		double errorY = targetPose.getY() - robotPose.getY();

		double ffX = Math.signum(errorX) * 0.05;
		double ffY = Math.signum(errorY) * 0.05;
		poseError = new com.acmerobotics.roadrunner.geometry.Pose2d(errorX,errorY,headingError);

		//yPower = -yPower;
		//double turnPower = clipPower(headingError * kpTurn) * powerScaler;
		//double turnPower = clipPower(turnController.calculate(0,-headingError)) * powerScalar;
		double turnPower = turnControlWrapped.calculate(targetAngle,angle);
		turnPower *= powerScalar;
		double headingFF = Math.signum(turnPower) * 0.03;

		double errorMag = poseError.vec().norm();
		errorMagDeriv = errorMag - previousErrorMag;
		errorMagDeriv /= timer.seconds();
		previousErrorMag = errorMag;
		timer.reset();

		return new com.acmerobotics.roadrunner.geometry.Pose2d(xPower + ffX,yPower + ffY,turnPower + headingFF);

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

	public Vector2d normalizePower(double xPower, double yPower) {
		Vector2d powerVec = new Vector2d(xPower,yPower);
//		if (Math.abs(xPower) > 0 || Math.abs(yPower) > 0) {
//			powerVec = powerVec.div(powerVec.norm());
//		}
		return powerVec;
	}

	public double headingErrorMag() {
		return Math.abs(poseError.getHeading());
	}

	protected double clipPower(double power) {
		return Range.clip(power,-1,1);
	}

	public double getErrorMagDeriv() {
		return Math.abs(errorMagDeriv);
	}
}
