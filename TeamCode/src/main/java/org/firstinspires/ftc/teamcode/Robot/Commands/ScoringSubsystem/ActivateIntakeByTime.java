package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

public class ActivateIntakeByTime extends Command {

	ElapsedTime timer = new ElapsedTime();
	ScoringMechanism mechanism;
	double durationS;


	public ActivateIntakeByTime(ScoringMechanism mechanism, double durationS) {
		super(mechanism);
		this.mechanism = mechanism;
		this.durationS = durationS;
	}

	@Override
	public void init() {
		mechanism.ACTIVATE_INTAKE();
		timer.reset();
	}

	@Override
	public void periodic() {
		mechanism.ACTIVATE_INTAKE();
	}

	@Override
	public boolean completed() {
		return timer.seconds() > durationS;
	}

	@Override
	public void shutdown() {
		mechanism.STOP_INTAKE();
	}
}
