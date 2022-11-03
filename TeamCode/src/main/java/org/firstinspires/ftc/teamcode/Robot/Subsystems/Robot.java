package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Simulation.TestCommandsSubsystems.PrintSubsystem1;

public class Robot {


	public enum OpMode {
		Auto,
		Teleop
	}

	public Dashboard dashboard = new Dashboard();
	public Input gamepad1;
	public Input gamepad2;
	public Drivetrain drivetrain = new Drivetrain();
	public PoleDetectionSubsystem detectionSubsystem = new PoleDetectionSubsystem(dashboard);
	public ScoringMechanism scoringMechanism = new ScoringMechanism();
	public DistanceSensor distanceSensor = new DistanceSensor();

	// print subsystem for testing
	public PrintSubsystem1 print = new PrintSubsystem1();

	protected CommandScheduler scheduler;

	public Robot(HardwareMap hwMap, OpMode opMode, Gamepad gamepad1, Gamepad gamepad2) {
		scheduler = new CommandScheduler(hwMap, drivetrain, dashboard, detectionSubsystem, scoringMechanism, distanceSensor);

		this.gamepad1 = new Input(gamepad1, scheduler);
		this.gamepad2 = new Input(gamepad2, scheduler);

		if (opMode.equals(OpMode.Auto)) {
			scheduler.initAuto();
		} else if (opMode.equals(OpMode.Teleop)) {
			scheduler.initTeleop();
		}
	}

	public void update() {
		updateInput();
		scheduler.run();
	}

	public void shutdown() {
		scheduler.shutdown();
	}

	public void updateInput() {
		gamepad1.periodic();
		gamepad2.periodic();
	}

	public CommandScheduler getScheduler() {
		return scheduler;
	}




}
