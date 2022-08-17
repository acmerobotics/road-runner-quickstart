package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseTeleop;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Simulation.TestCommandsSubsystems.PrintCommand1;


@TeleOp
public class TestTeleop extends BaseTeleop {

	@Override
	public Command setupTeleop(CommandScheduler scheduler) {
		Command printCommand = new PrintCommand1(robot.print, "Hello, World!");
		robot.gamepad1.whenCrossPressed(printCommand);

		//return new ClosedLoopTeleop(robot.drivetrain,robot.odometry,robot.gamepad1);
		return null;
	}
}
