package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseTeleop;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RobotRelative;
import org.firstinspires.ftc.teamcode.Simulation.TestCommandsSubsystems.PrintCommand1;


@TeleOp
public class SetServoPosition extends BaseTeleop {

	@Override
	public Command setupTeleop(CommandScheduler scheduler) {
		robot.arm.setWristPosition(0);

		return new RobotRelative(robot, robot.gamepad1);
	}
}
