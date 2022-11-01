package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseTeleop;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.AutoAlignWithVision;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RobotRelative;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem.ActivateIntakeByTime;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem.ActivateIntakeToggle;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem.Deposit;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem.GoToScore;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Intake_prototype_1;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;
import org.firstinspires.ftc.teamcode.Simulation.TestCommandsSubsystems.PrintCommand1;

import java.util.function.BooleanSupplier;


@TeleOp
public class TestTeleop extends BaseTeleop {

	@Override
	public Command setupTeleop(CommandScheduler scheduler) {
		BooleanSupplier intakeSupplier = null;

		if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.N) {
			intakeSupplier = () -> gamepad1.left_bumper;
		}

		robot.gamepad1.whenLeftBumperPressed(new ActivateIntakeToggle(robot.scoringMechanism, gamepad1, intakeSupplier));
		robot.gamepad1.whenTrianglePressed(new GoToScore(robot.scoringMechanism, ScoringMechanism.States.HIGH));
		robot.gamepad1.whenSquarePressed(new GoToScore(robot.scoringMechanism, ScoringMechanism.States.MID));
		robot.gamepad1.whenRightBumperPressed(new Deposit(robot.scoringMechanism));
		robot.gamepad1.whenLeftStickButtonPressed(new AutoAlignWithVision(robot.drivetrain,robot.detectionSubsystem)
				.addNext(new RobotRelative(robot,robot.gamepad1)));

		//return new ClosedLoopTeleop(robot.drivetrain,robot.odometry,robot.gamepad1);
		return new RobotRelative(robot, robot.gamepad1);
	}
}
