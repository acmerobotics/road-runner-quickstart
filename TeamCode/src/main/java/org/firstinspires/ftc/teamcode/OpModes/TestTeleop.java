package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseTeleop;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RobotRelative;
import org.firstinspires.ftc.teamcode.Robot.Commands.Intakes.SetIntake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake_prototype_1;
import org.firstinspires.ftc.teamcode.Simulation.TestCommandsSubsystems.PrintCommand1;


@TeleOp
public class TestTeleop extends BaseTeleop {

	@Override
	public Command setupTeleop(CommandScheduler scheduler) {
		Command printCommand = new PrintCommand1(robot.print, "Hello, World!");
		robot.gamepad1.whenCrossPressed(printCommand);
		Command intake_on = new SetIntake(robot.intake, Intake_prototype_1.INTAKE_STATES.ON);
		Command intake_off = new SetIntake(robot.intake, Intake_prototype_1.INTAKE_STATES.OFF);
		Command intake_reverse = new SetIntake(robot.intake, Intake_prototype_1.INTAKE_STATES.REVERSED);

		robot.gamepad1.whenCirclePressed(intake_off);
		robot.gamepad1.whenCrossPressed(intake_on);
		robot.gamepad1.whenTrianglePressed(intake_reverse);

		//return new ClosedLoopTeleop(robot.drivetrain,robot.odometry,robot.gamepad1);
		return new RobotRelative(robot, robot.gamepad1);
	}
}
