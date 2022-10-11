package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.FeedbackController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.NoFeedback;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;

import java.util.HashMap;

public class TestArm extends Subsystem {

	protected DcMotorEx bottom_lift;
	protected DcMotorEx middle_lift;
	protected Servo top_lift;
	protected Servo claw;

	protected double bottom_setpoint = 0;
	protected double mid_setpoint = 0;

	FeedbackController bottom_controller = new NoFeedback();
	FeedbackController middle_controller = new NoFeedback();

	protected ArmStates state = ArmStates.IN;

	HashMap<ArmStates, Double> bottom_map = new HashMap<>();
	HashMap<ArmStates, Double> mid_map = new HashMap<>();
	HashMap<ArmStates, Double> top_map = new HashMap<>();
	HashMap<ArmStates, Double> claw_map = new HashMap<>();

	ElapsedTime timer = new ElapsedTime();

	boolean firstRun = true;

	public TestArm() {

		// hashmap for bottoms
		bottom_map.put(ArmStates.IN,0.0);
		bottom_map.put(ArmStates.CARRY,0.0);
		bottom_map.put(ArmStates.LOW,0.0);
		bottom_map.put(ArmStates.TOP,0.0);
		bottom_map.put(ArmStates.COLLECT,0.0);
		bottom_map.put(ArmStates.TOP_PLACE,0.0);
		bottom_map.put(ArmStates.LOW_PLACE,0.0);

		// mid hashmap
		mid_map.put(ArmStates.IN,0.0);
		mid_map.put(ArmStates.CARRY,0.0);
		mid_map.put(ArmStates.LOW,0.0);
		mid_map.put(ArmStates.TOP,0.0);
		mid_map.put(ArmStates.COLLECT,0.0);
		mid_map.put(ArmStates.TOP_PLACE,0.0);
		mid_map.put(ArmStates.LOW_PLACE,0.0);

		// maps for top
		top_map.put(ArmStates.IN,0.0);
		top_map.put(ArmStates.CARRY,0.0);
		top_map.put(ArmStates.LOW,0.0);
		top_map.put(ArmStates.TOP,0.0);
		top_map.put(ArmStates.COLLECT,0.0);
		top_map.put(ArmStates.TOP_PLACE,0.0);
		top_map.put(ArmStates.LOW_PLACE,0.0);

		// maps for claw
		claw_map.put(ArmStates.IN,0.0);
		claw_map.put(ArmStates.CARRY,0.0);
		claw_map.put(ArmStates.LOW,0.0);
		claw_map.put(ArmStates.TOP,0.0);
		claw_map.put(ArmStates.COLLECT,0.0);
		claw_map.put(ArmStates.TOP_PLACE,0.0);
		claw_map.put(ArmStates.LOW_PLACE,0.0);


	}


	private void basic_init(HardwareMap hwmap) {
		bottom_lift = hwmap.get(DcMotorEx.class, "bottom_lift");
		middle_lift = hwmap.get(DcMotorEx.class, "mid_lift");
		top_lift = hwmap.get(Servo.class, "top_extension");
		claw = hwmap.get(Servo.class, "claw");
	}

	@Override
	public void initAuto(HardwareMap hwMap) {
		basic_init(hwMap);
		bottom_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		middle_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		top_lift.setPosition(fixNull(top_map.get(ArmStates.IN),"set position in init for top_lift"));
		claw.setPosition(fixNull(claw_map.get(ArmStates.IN), "set position in init for claw"));
	}


	@Override
	public void initTeleop(HardwareMap hwMap) {
		basic_init(hwMap);
	}

	public void periodic() {

		if (firstRun) {
			firstRun = false;
			timer.reset();
		}

		switch (state) {
			case IN:

				break;
			case DEPLOY:
				break;
			case CARRY:
				break;
			case COLLECT:
				break;
			case LOW:
				break;
			case TOP:
				break;
			case LOW_PLACE:
				break;
			case TOP_PLACE:
				break;
		}

		setPositions(state);


	}

	@Override
	public void shutdown() {

	}


	public enum ArmStates {
		IN,
		DEPLOY,
		CARRY,
		COLLECT,
		LOW,
		TOP,
		LOW_PLACE,
		TOP_PLACE
	}

	private void setPositions(ArmStates current_state) {
		claw.setPosition(fixNull(claw_map.get(current_state),"claw"));
		top_lift.setPosition(fixNull(top_map.get(current_state), "top of arm"));
		bottom_setpoint = fixNull(bottom_map.get(current_state), "bottom of arm");
		mid_setpoint = fixNull(mid_map.get(current_state), "middle of arm");
	}

	/*
		We are 10x developers, our code will never crash.
	 */
	public double fixNull(Double value, String accessLabel) {
		if (value == null) {
			System.out.println(accessLabel + " is throwing null pointer exceptions, returning 0 as a contingency, likely a value is not set for its appropriate hashmap");
			return 0;
		}
		return value;
	}
}
