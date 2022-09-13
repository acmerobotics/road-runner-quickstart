package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;

public class Intake_prototype_1 extends Subsystem {

	protected DcMotorEx left_intake;
	protected DcMotorEx right_intake;
	protected INTAKE_STATES state = INTAKE_STATES.OFF;

	@Override
	public void initAuto(HardwareMap hwMap) {

		left_intake = hwMap.get(DcMotorEx.class, "intake_left");
		right_intake = hwMap.get(DcMotorEx.class, "intake_right");
		right_intake.setDirection(DcMotorSimple.Direction.REVERSE);

	}

	@Override
	public void periodic() {
		setPower(state.getPower());
	}

	@Override
	public void shutdown() {
		state = INTAKE_STATES.OFF;
	}

	protected void setPower(double power) {
		left_intake.setPower(power);
		right_intake.setPower(power);
	}

	public void setPower(INTAKE_STATES state) {
		this.state = state;
	}


	public enum INTAKE_STATES {

		ON(1),
		OFF(0.3),
		REVERSED(-0.4);

		protected double power;

		INTAKE_STATES(double i) {
			this.power = i;
		}

		public double getPower() {
			return power;
		}
	}

}
