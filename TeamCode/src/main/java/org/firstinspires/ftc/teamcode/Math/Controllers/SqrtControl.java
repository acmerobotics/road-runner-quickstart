package org.firstinspires.ftc.teamcode.Math.Controllers;


import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.FeedbackController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Math.Controllers.Coefficient.SqrtCoefficients;

public class SqrtControl implements FeedbackController {

	SqrtCoefficients coefficients;

	ElapsedTime timer = new ElapsedTime();
	private double errorPrevious = 0;
	boolean hasRun = false;

	public SqrtControl(SqrtCoefficients coefficients) {
		this.coefficients = coefficients;
		timer.reset();
	}

	@Override
	public double calculate(double reference, double state) {
		double error = reference - state;
		double output;
		if (error > 0) {
			output = Math.sqrt(error) * coefficients.getK() + coefficients.getH();
		} else {
			output = -Math.sqrt(Math.abs(error)) * coefficients.getK() - coefficients.getH();
		}
		checkForStart();
		double derivative =( error - errorPrevious) / timer.seconds();
		timer.reset();
		errorPrevious = error;

		return output + coefficients.getKd() * derivative;
	}

	public void checkForStart() {
		if (!hasRun) {
			hasRun = true;
		}
	}


	public void setCoefficients(SqrtCoefficients coefficients) {
		this.coefficients = coefficients;
	}
}

