package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.Math.AsymmetricProfile.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.Math.AsymmetricProfile.MotionConstraint;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;

public class ProfiledServo extends Subsystem {
	public Servo servo_right;
	public Servo servo_left;
	protected double endPosition;
	protected double previousEndPosition;
	protected double currentPosition;
	protected String name;

	public AsymmetricMotionProfile profile_m;
	public MotionConstraint constraint;
	public ElapsedTime timer = new ElapsedTime();

	public ProfiledServo(HardwareMap hwmap, String name1, String name2, double velo, double accel, double decel, double initialPosition) {
		servo_left = hwmap.get(Servo.class, name1);
		servo_right = hwmap.get(Servo.class,name2);
		this.name = name1 + " " + name2 + " ";
		this.endPosition = initialPosition;
		this.currentPosition = initialPosition;
		this.previousEndPosition = initialPosition + 100; // just guarantee that they are not equal
 		this.constraint = new MotionConstraint(velo,accel,decel);
		setPositionsSynced(initialPosition);
	}

	protected void regenerate_profile() {
		profile_m = new AsymmetricMotionProfile(this.currentPosition,this.endPosition,this.constraint);
		timer.reset();
	}

	@Override
	public void initAuto(HardwareMap hwMap) {

	}

	@Override
	public void periodic() {
		if (endPosition != previousEndPosition) {
			regenerate_profile();
		}
		previousEndPosition = endPosition;
		double current_target = profile_m.calculate(timer.seconds()).getX();
		setPositionsSynced(current_target);
		Dashboard.packet.put(name + "position: ", current_target);
	}

	public boolean isBusy() {
		return timer.seconds() < profile_m.getProfileDuration();
	}

	@Override
	public void shutdown() {

	}

	public void setPosition(double endPosition) {
		this.endPosition = endPosition;
	}

	protected void setPositionsSynced(double pos) {
		servo_left.setPosition(1 - pos);
		servo_right.setPosition(pos);
	}
}
