package org.firstinspires.ftc.teamcode.Math.Controllers;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Utils.Vector;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.Robot.ControlConstants;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Math.AsymmetricProfile.AsymmetricMotionProfile;

import java.util.function.DoubleSupplier;

public class DistanceDriveControl {

	BasicPID distanceControl = new BasicPID(ControlConstants.distanceControl);
	AsymmetricMotionProfile profile_n = new AsymmetricMotionProfile(10,10,ControlConstants.driveConstraintsNew);;
	TurnOnlyControl turnControl;
	ElapsedTime timer = new ElapsedTime();

	double trackingError = 0;
	double endPoseError = 0;
	double previousReference = 0;
	double reference_p = 0;
	boolean hasRun = false;

	public DistanceDriveControl(DoubleSupplier robotAngle, double headingReference) {
		turnControl = new TurnOnlyControl(robotAngle, headingReference,true);
	}

	/**
	 * returns the wheel powers as a vector
	 * @param reference target distance
	 * @param state current traveled distance
	 * @return 2 state vector, item 0 is left, item 1 is right
	 */
	@RequiresApi(api = Build.VERSION_CODES.N)
	public Vector calculate(double reference, double state) {

		double direction = Math.signum(reference);
		if (direction == 0) direction = 1;
		reference = Math.abs(reference);
		regenerateProfile(reference,state);
		reference_p = profile_n.calculate(timer.seconds()).getX();
		trackingError = reference_p - state;
		endPoseError = reference - state;

		telemetry(reference, state);
		Vector output = new Vector(2);

		double forward = distanceControl.calculate(reference_p,state) * direction;
		Vector turn = turnControl.calculate();
		double scalar = getTurnScalar();
		output.set(forward * scalar, 0);
		output.set(forward * scalar, 1);

		try {
			return output.add(turn);
		} catch (Exception e) {
			e.printStackTrace();
			return output;
		}
	}

	private void telemetry(double reference, double state) {
		Dashboard.packet.put("Target distance",reference);
		Dashboard.packet.put("profile distance",reference_p);
		Dashboard.packet.put("Measured distance", state);
	}

	protected double getTurnScalar() {
		return Math.cos(Range.clip(turnControl.getEndGoalError(),-Math.PI / 2, Math.PI / 2));
	}


	public double endPoseError() {
		return endPoseError;
	}

	public void regenerateProfile(double reference, double state) {
		if (reference != previousReference || !hasRun) {
			hasRun = true;
			profile_n = new AsymmetricMotionProfile(state, reference, ControlConstants.driveConstraintsNew);
			timer.reset();
		}
		previousReference = reference;
	}


}
