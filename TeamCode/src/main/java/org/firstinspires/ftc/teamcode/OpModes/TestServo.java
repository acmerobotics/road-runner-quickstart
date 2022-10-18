package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseTeleop;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RobotRelative;


@TeleOp
public class TestServo extends BaseTeleop {

	@Override
	public Command setupTeleop(CommandScheduler scheduler) {
		hardwareMap.get(Servo.class, "wrist").setPosition(0.1);

		return new RobotRelative(robot, robot.gamepad1);
	}
}
