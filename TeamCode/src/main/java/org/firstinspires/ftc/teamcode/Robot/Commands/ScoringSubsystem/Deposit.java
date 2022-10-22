package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

public class Deposit extends Command {

	boolean invalid_start = false;
	ScoringMechanism mechanism;

	public Deposit(ScoringMechanism mechanism) {
		this.mechanism = mechanism;
	}

	@Override
	public void init() {
		invalid_start = !mechanism.getState().equals(ScoringMechanism.States.HIGH)
				&& !mechanism.getState().equals(ScoringMechanism.States.MID)
				&& !mechanism.getState().equals(ScoringMechanism.States.LOW);
		mechanism.activateEjection();
		System.out.println("State at beginning of Deposit is " + mechanism.getState());
		System.out.println("did Deposit begin with invalid start? " + invalid_start);
	}

	@Override
	public void periodic() {

	}

	@Override
	public boolean completed() {
		return mechanism.getState().equals(ScoringMechanism.States.GO_TO_INTAKE) ||
				mechanism.getState().equals(ScoringMechanism.States.CARRY) ;
	}

	@Override
	public void shutdown() {

	}
}
