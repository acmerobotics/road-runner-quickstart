package org.firstinspires.ftc.teamcode.CommandFramework;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Subsystems.Robot;

public abstract class BaseAuto extends LinearOpMode {

	protected Robot robot;

	@Override
	public void runOpMode() {
		robot = new Robot(hardwareMap, Robot.OpMode.Auto, gamepad1, gamepad2);
		waitForStart();

		robot.getScheduler().forceCommand(setupAuto(robot.getScheduler()));

		while (opModeIsActive()) {
			robot.update();

		}

	}

	public abstract Command setupAuto(CommandScheduler scheduler);




}
