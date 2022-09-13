package org.firstinspires.ftc.teamcode.Robot.Commands.Intakes;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake_prototype_1;

public class SetIntake extends Command {

	protected Intake_prototype_1.INTAKE_STATES state;
	protected Intake_prototype_1 intake;
	public boolean hasSet = false;



	public SetIntake(Intake_prototype_1 intake, Intake_prototype_1.INTAKE_STATES state) {
		super(intake);
		this.state = state;
		this.intake = intake;
	}


	@Override
	public void init() {
		intake.setPower(state);
		hasSet = true;
	}

	@Override
	public void periodic() {

	}

	@Override
	public boolean completed() {
		return hasSet;
	}

	@Override
	public void shutdown() {

	}
}
