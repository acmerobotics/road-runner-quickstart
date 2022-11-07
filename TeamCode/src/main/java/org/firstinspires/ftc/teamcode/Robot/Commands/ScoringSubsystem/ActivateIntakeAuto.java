package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

public class ActivateIntakeAuto extends Command {
	ScoringMechanism mechanism;
	protected boolean invalidStart = false;

	public ActivateIntakeAuto(ScoringMechanism mechanism) {
		super(mechanism);
		this.mechanism = mechanism;
	}

	@Override
	public void init() {

		if (!mechanism.getState().equals(ScoringMechanism.States.AUTO_INTAKE_SAFE)) {
			invalidStart = true;
		}
		System.out.println("Did Activate intake Auto had an invalid start?" + invalidStart);

		mechanism.ACTIVATE_INTAKE_AUTO();

	}

	@Override
	public void periodic() {

	}

	@Override
	public boolean completed() {
		return mechanism.getState().equals(ScoringMechanism.States.AUTO_STOP_IN_TAKING) || invalidStart;
	}

	@Override
	public void shutdown() {

	}
}
