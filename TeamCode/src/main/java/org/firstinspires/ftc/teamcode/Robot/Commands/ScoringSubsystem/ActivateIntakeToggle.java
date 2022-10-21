package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

public class ActivateIntakeToggle extends Command {

	ScoringMechanism mechanism;
	Gamepad gamepad;

	public ActivateIntakeToggle(ScoringMechanism mechanism, Gamepad gamepad) {
		super(mechanism);
		this.mechanism = mechanism;
		this.gamepad = gamepad;
	}

	@Override
	public void init() {

	}

	@Override
	public void periodic() {
		mechanism.ACTIVATE_INTAKE();
	}

	@Override
	public boolean completed() {
		return !gamepad.cross;
	}

	@Override
	public void shutdown() {
		mechanism.STOP_INTAKE();
	}
}
