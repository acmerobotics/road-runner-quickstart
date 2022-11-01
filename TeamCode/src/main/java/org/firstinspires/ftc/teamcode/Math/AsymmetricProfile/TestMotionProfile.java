package org.firstinspires.ftc.teamcode.Math.AsymmetricProfile;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TestMotionProfile {

	public static void main(String[] args) {
		double desiredPosition = 0;
		MotionConstraint slide_constraints = new MotionConstraint(2500,2500,750);
		AsymmetricMotionProfile profile = new AsymmetricMotionProfile(400,0,slide_constraints);
		ElapsedTime timer = new ElapsedTime();
		double state = profile.calculate(timer.seconds()).getX();
		while (state > desiredPosition) {
			state = profile.calculate(timer.seconds()).getX();
			System.out.println("position is: " + state);
		}


	}
}
