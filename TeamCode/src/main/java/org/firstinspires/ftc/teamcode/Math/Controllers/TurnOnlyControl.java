package org.firstinspires.ftc.teamcode.Math.Controllers;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Utils.MathUtils;
import com.ThermalEquilibrium.homeostasis.Utils.Vector;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.ControlConstants;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Math.AsymmetricProfile.AsymmetricMotionProfile;

import java.util.function.DoubleSupplier;

import static org.firstinspires.ftc.teamcode.Robot.Subsystems.Robot.IS_NEW_6wd;

public class TurnOnlyControl {

	protected double headingReference;
	protected double endGoalError = 0;
	protected double previousReference = 0;

	protected DoubleSupplier robotAngle;


	boolean hasRun = false;

	ElapsedTime timer = new ElapsedTime();
	AsymmetricMotionProfile profile_n;


	SqrtControl angleController;

	AngleController angleControl;

	double trackingError = 0;

	public TurnOnlyControl(DoubleSupplier robotAngle, double headingReference) {
		this.robotAngle = robotAngle;
		this.headingReference = headingReference;
		if (IS_NEW_6wd) {
			angleController = new SqrtControl(ControlConstants.angleControl2);
		} else {
			angleController = new SqrtControl(ControlConstants.angleControl);
		}
		angleControl = new AngleController(angleController);
	}

	public TurnOnlyControl(DoubleSupplier robotAngle, double headingReference, boolean forDriving) {
		this.robotAngle = robotAngle;
		this.headingReference = headingReference;
		if (IS_NEW_6wd) {
			if (forDriving) {
				angleController = new SqrtControl(ControlConstants.angleControl3);
			} else {
				angleController = new SqrtControl(ControlConstants.angleControl2);
			}
		} else {
			angleController = new SqrtControl(ControlConstants.angleControl);
		}
		angleControl = new AngleController(angleController);
	}
	/**
	 * returns the wheel powers as a vector
	 * @return 2 state vector, item 0 is left, item 1 is right
	 */
	@RequiresApi(api = Build.VERSION_CODES.N)
	public Vector calculate() {

		regenerateProfile(robotAngle.getAsDouble(), headingReference);
		double profileState = profile_n.calculate(timer.seconds()).getX();
		Vector output = new Vector(2);

		endGoalError = MathUtils.normalizedHeadingError(headingReference, robotAngle.getAsDouble());
		trackingError = MathUtils.normalizedHeadingError(profileState, robotAngle.getAsDouble());

		telemetry();

		double heading = -angleControl.calculate(0,  endGoalError);

		double left = + heading;
		double right = - heading;

		output.set(left, 0);
		output.set(right, 1);

		return output;
	}

	public double getEndGoalError() {
		return endGoalError;
	}


	public void regenerateProfile(double reference, double state) {
		if (reference != previousReference || !hasRun) {
			hasRun = true;
			profile_n = new AsymmetricMotionProfile(
					MathUtils.normalizedHeadingError(headingReference, state),
					0,
					ControlConstants.turnConstraintsNew);
			timer.reset();
		}
		previousReference = reference;
	}

	@RequiresApi(api = Build.VERSION_CODES.N)
	protected void telemetry() {
		Dashboard.packet.put("Turn Tracking Error", trackingError);
		Dashboard.packet.put("End Goal Error", endGoalError);
		Dashboard.packet.put("Turn reference", headingReference);
		Dashboard.packet.put("Robot Angle", robotAngle.getAsDouble());
	}
}
