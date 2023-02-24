package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

public class SmartdampPID {
	// full paper: https://docs.google.com/document/d/1z8FsP15JZzCeDjgSEzDGUuL1snXx7JuwIbqEBelWHRM/edit?usp=sharing
	// shorter TLDR: https://docs.google.com/document/d/1dvL3qKIGrDjgbJeLS64o7MmCLA-OyQ0qxphs8Y7Ty1A/edit

	/**
	 * Generates Critically Damped Derivative Gains given Kv,Ka, and a complementary proportional Gain
	 * @param kP your proportional gain
	 * @param rotation true if these coefficients are for the rotational axis, false if for translation.
	 * @return critically damped derivative gain if kP is not already under-damped.
	 */
	public static double generateSMARTDerivativeTerm(double kP, boolean rotation) {

		double kV = DriveConstants.kV;
		double kA = DriveConstants.kA;


		if (rotation) {
			kV /= DriveConstants.TRACK_WIDTH;
			kA /= DriveConstants.TRACK_WIDTH;
		}

		// anything below this will result in a non minimum phase system.
		// non-minimum phase means the system will hesitate, similar to that of a turning bicycle or a pitching aircraft.
		// while it would technically be faster, I have not yet proved it's stability so I will leave it out for now.
		double criticalKp = (kV * kV) / (4 * kA);
		if (criticalKp >= kP) {
			return 0;
		}
		// the critically damped PID derivative gain.
		return 2 * Math.sqrt(kA * kP) - kV;
	}

	public static PIDCoefficients TranslationCoefficientsSMART(double kP) {
		return new PIDCoefficients(kP,0,generateSMARTDerivativeTerm(kP,false));
	}
	public static PIDCoefficients RotationCoefficientsSMART(double kP) {
		return new PIDCoefficients(kP,0,generateSMARTDerivativeTerm(kP,true));
	}
}
