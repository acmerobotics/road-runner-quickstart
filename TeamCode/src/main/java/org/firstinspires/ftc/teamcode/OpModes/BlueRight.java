package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem.Deposit;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem.GoToScore;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

@Autonomous
public class BlueRight extends BaseAuto {

	public final Pose2d initialPose = new Pose2d( -36, 63.5, Math.toRadians(-90));
	@Override
	public Command setupAuto(CommandScheduler scheduler) {

//
		Trajectory traj1 = robot.drivetrain.getBuilder().trajectoryBuilder(initialPose)
			.lineToLinearHeading(new Pose2d(-36.0, 18, Math.toRadians(-90)))
				.build();

		Trajectory traj2 = robot.drivetrain.getBuilder().trajectoryBuilder(traj1.end())
				.lineToLinearHeading(new Pose2d(-25, 11.5, Math.toRadians(-65)))
				.build();



		return follow(traj1)
				.addNext(new MultipleCommand(new GoToScore(robot.scoringMechanism, ScoringMechanism.States.HIGH), follow(traj2)))
				.addNext(new Deposit(robot.scoringMechanism));


	}

	@Override
	public void setRobotPosition() {
		robot.drivetrain.setPose(initialPose);
	}
}
