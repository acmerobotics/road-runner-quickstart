package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

public class GoToScore extends Command {

	ScoringMechanism.States targetState;
	ScoringMechanism mechanism;

	ScoringMechanism.States previousState;

	ElapsedTime timer = new ElapsedTime();

	double arm_move_time = 1;

	boolean invalidStart = false;

	public GoToScore(ScoringMechanism mechanism, ScoringMechanism.States state) {
		super(mechanism);
		this.targetState = state;
		this.mechanism = mechanism;
	}

	@Override
	public void init() {
		mechanism.setTarget(targetState);
		mechanism.GO_TO_SCORING();
		if (mechanism.getState().equals(ScoringMechanism.States.CARRY) || mechanism.getState().equals(ScoringMechanism.States.INTAKE_ON)) {
			invalidStart = true;
		}
		previousState = mechanism.getState();
		System.out.println("State during Go To Score is: " + previousState);


	}

	@Override
	public void periodic() {

		if (!previousState.equals(mechanism.getState())) {
			timer.reset();
		}

		previousState = mechanism.getState();
	}

	@Override
	public boolean completed() {
		return (timer.seconds() > arm_move_time && (mechanism.getState().equals(targetState))) || invalidStart;
	}

	@Override
	public void shutdown() {

	}
}
