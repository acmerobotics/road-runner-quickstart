package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;

public class QuasiStaticVelocity extends Command {
	Drivetrain drivetrain;
	ElapsedTime timer = new ElapsedTime();

	double rampRate;
	double maxLength;

	public QuasiStaticVelocity(Drivetrain drivetrain, double rampRate, double maxLength) {
		super(drivetrain);

		this.drivetrain = drivetrain;
		this.rampRate = rampRate;
		this.maxLength = maxLength;
	}


	@Override
	public void init() {
		timer.reset();
		drivetrain.robotRelative(new Pose2d(0,0,0));
	}

	@Override
	public void periodic() {
		double position = drivetrain.getPose().getX();
		double velocity = drivetrain.getVelocity().getX();
		double newPower = Range.clip(timer.seconds() * rampRate,0.00,1);

		drivetrain.robotRelative(new Pose2d(newPower,0,0));

		RobotLog.ii("SysID (P/V/u/t): ", position + " , " + velocity + " , " + newPower + " , " + timer.seconds());
	}

	@Override
	public boolean completed() {
		return timer.seconds() > maxLength;
	}

	@Override
	public void shutdown() {
		drivetrain.robotRelative(new Pose2d(0,0,0));
	}
}

