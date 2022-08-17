package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Input;

public class CheesyDrive extends Command {

	Drivetrain drivetrain;
	Input gamepad1;

	double deadBand = 0.1;
	double turnGain = 1.25;

	public CheesyDrive(Input gamepad,Drivetrain drivetrain) {
		super(drivetrain, gamepad);
		this.drivetrain = drivetrain;
		this.gamepad1 = gamepad;
	}
	@Override
	public void init() {

	}

	@Override
	public void periodic() {
		double forwardRaw = gamepad1.getForwardJoystick();
		double turnRaw = gamepad1.getTurnJoystick();

		double turnAmount;
		double forwardAmount = 0;

		if (Math.abs(forwardRaw) < deadBand) {
			turnAmount = turnRaw;
		} else {
			turnAmount = Math.abs(forwardRaw) * turnRaw * turnGain;
			forwardAmount = forwardRaw;
		}



		drivetrain.robotRelative(forwardAmount, turnAmount);



	}

	@Override
	public boolean completed() {
		return false;
	}

	@Override
	public void shutdown() {

	}
}
