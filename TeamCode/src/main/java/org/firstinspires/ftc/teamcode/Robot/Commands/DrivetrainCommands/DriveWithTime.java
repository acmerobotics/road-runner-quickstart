package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;

public class DriveWithTime extends Command {

	Drivetrain drivetrain;

	ElapsedTime timer = new ElapsedTime();

	double power;
	double seconds;

	public DriveWithTime(Drivetrain drivetrain, double seconds, double power) {
		super(drivetrain);
		this.power = power;
		this.drivetrain = drivetrain;
		this.seconds = seconds;
	}


	@Override
	public void init() {
		timer.reset();
		drivetrain.robotRelative(power,0);
	}

	@Override
	public void periodic() {

	}

	@Override
	public boolean completed() {
		return timer.seconds() > seconds;
	}

	@Override
	public void shutdown() {
		drivetrain.setPower(0,0);
	}
}
