package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;

@Autonomous
public class PushingAuto extends BaseAuto {
	@Override
	public Command setupAuto(CommandScheduler scheduler) {
		return new DriveWithTime(robot.drivetrain,10,1);
	}
}
