package org.firstinspires.ftc.teamcode.returnToUltimateGoal;

import com.qualcomm.robotcore.util.ElapsedTime;

public class OneDimensionalLQRController {

	// reference signal scaling term
	protected double Kr = 0;

	// gain 'matrix' K
	protected double K = 0;

	// output gain scaler
	protected double outputGain = 1;

	private double integral_sum_max = 1;
	private double integral_sum_min = -integral_sum_max;

	private double integral_sum = 0;

	private double Ki = 0;

	private double last_setpoint = 0;


	double kd = 0.12627 * 0.06;

	double last_error = 0;
	double error = 0;


	ElapsedTime timer = new ElapsedTime();
	/**
	 * constructor pogg
	 * @param K gain K
	 * @param Kr signal scaling
	 */
	public OneDimensionalLQRController(double K, double Kr, double Ki) {
		this.Kr = Kr;
		this.K = K;
		this.outputGain = 1;
		this.Ki = Ki;
	}

	/**
	 * constructor with output scaling pogg
	 * @param K gain K
	 * @param Kr signal scaling
	 * @param outputGain
	 */
	public OneDimensionalLQRController(double K, double Kr, double outputGain, double Ki) {
		this.K = K;
		this.Kr = Kr;
		this.outputGain = outputGain;
		this.Ki = Ki;
	}
	/**
	 * outputs the signal from the LQR controller
	 * @param reference the state we want to achieve
	 * @param state the state we are at
	 * @return the command to send to the motor
	 */
	public double outputLQR(double reference, double state) {

		if (reference != last_setpoint) {
			integral_sum = 0;
		}

		double scaledReference = reference * Kr;
		double scaledState = state * K;
		error = scaledReference - scaledState;

		integral_sum += (reference - state) * timer.milliseconds();
		if (integral_sum_max < integral_sum) {
			integral_sum = integral_sum_max;
		}
		if (integral_sum_min > integral_sum) {
			integral_sum = integral_sum_min;
		}

		double state_error = (reference - state);
		double derivative = state_error - last_error / timer.milliseconds();
		last_error = (reference - state);

		last_setpoint = reference;

		timer.reset();
		return (error * outputGain) + (integral_sum * Ki) + (derivative *kd);

	}

	public double getError() {
		return error;
	}

	public double getK() {
		return K;
	}

	public double getKr() {
		return Kr;
	}

	public double getOutputGain() {
		return outputGain;
	}


	public void setK(double k) {
		K = k;
	}

	public void setKr(double kr) {
		Kr = kr;
	}

	public void setOutputGain(double outputGain) {
		this.outputGain = outputGain;
	}




}
