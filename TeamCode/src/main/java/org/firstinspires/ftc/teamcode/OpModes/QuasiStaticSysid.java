package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.QuasiStaticVelocity;

@Autonomous
public class QuasiStaticSysid extends BaseAuto {
	@Override
	public Command setupAuto(CommandScheduler scheduler) {
		Command auto = new QuasiStaticVelocity(robot.drivetrain, 0.20, 10);
		return auto;
	}
}
