package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

public class GoToScore extends Command {

	ScoringMechanism.States targetState;
	ScoringMechanism mechanism;

	ScoringMechanism.States previousState;

	ElapsedTime timer = new ElapsedTime();

	public boolean hasStarted = false;

	double arm_move_time = 3;

	boolean invalidStart = true;

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
			invalidStart = false;
		}
		previousState = mechanism.getState();
		System.out.println("State during Go To Score is: " + previousState);
		System.out.println("GoToScore began with invalid start? "+  invalidStart);
		timer.reset();
	}

	@Override
	public void periodic() {

		hasStarted = true;
//		if (!previousState.equals(mechanism.getState())) {
//			timer.reset();
//		}

		previousState = mechanism.getState();
	}

	@Override
	public boolean completed() {

		boolean timerCondition = timer.seconds() > arm_move_time;
		boolean atTargetState = mechanism.getState().equals(targetState);

		System.out.println("Go To score conditions;;;; Timer condition is: " + timerCondition + " at target state is: " + atTargetState + " invalid start is: " + invalidStart);

		boolean primaryCondition = hasStarted && (timer.seconds() > arm_move_time && (mechanism.getState().equals(targetState)));

		return primaryCondition || invalidStart;
	}

	@Override
	public void shutdown() {

	}
}
