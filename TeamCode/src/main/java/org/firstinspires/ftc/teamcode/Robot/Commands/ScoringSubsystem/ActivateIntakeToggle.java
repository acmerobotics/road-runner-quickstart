package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem;

import android.os.Build;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import java.util.function.BooleanSupplier;

public class ActivateIntakeToggle extends Command {

	ScoringMechanism mechanism;
	Gamepad gamepad;
	BooleanSupplier button;

	public ActivateIntakeToggle(ScoringMechanism mechanism, Gamepad gamepad, BooleanSupplier button) {
		super(mechanism);
		this.mechanism = mechanism;
		this.gamepad = gamepad;
		this.button = button;
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
		if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
			return !this.button.getAsBoolean();
		}
		return true;
	}

	@Override
	public void shutdown() {
		mechanism.STOP_INTAKE();
	}
}
