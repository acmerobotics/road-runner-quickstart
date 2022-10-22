package org.firstinspires.ftc.teamcode.Math.AsymmetricProfile;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TestMotionProfile {

	public static void main(String[] args) {
		double desiredPosition = 14;
		MotionConstraint slide_constraints = new MotionConstraint(20,20,20);
		AsymmetricMotionProfile profile = new AsymmetricMotionProfile(0,14,slide_constraints);
		ElapsedTime timer = new ElapsedTime();
		double state = profile.calculate(timer.seconds()).getX();
		while (state < desiredPosition) {
			state = profile.calculate(timer.seconds()).getX();
			System.out.println("position is: " + state);
		}
		profile = new AsymmetricMotionProfile(desiredPosition,0,slide_constraints);
		timer.reset();
		while (state > 0) {
			state = profile.calculate(timer.seconds()).getX();
			System.out.println("position is: " + state);
		}

	}
}
