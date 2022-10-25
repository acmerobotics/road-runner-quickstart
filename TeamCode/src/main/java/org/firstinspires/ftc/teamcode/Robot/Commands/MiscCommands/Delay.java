package org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;

public class Delay extends Command {


	ElapsedTime timer = new ElapsedTime();


	double time;

	public Delay(double seconds) {
		this.time = seconds;
	}

	@Override
	public void init() {
		timer.reset();
	}

	@Override
	public void periodic() {

	}

	@Override
	public boolean completed() {
		return timer.seconds() > time;
	}

	@Override
	public void shutdown() {

	}
}
