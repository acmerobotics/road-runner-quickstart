package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.RR_quickstart.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RR_quickstart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.AlignWithVision2Auto;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.Delay;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem.ActivateIntakeAuto;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem.Deposit;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem.DepositAuto;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringSubsystem.GoToScore;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

@Autonomous
public class BlueRight extends BaseAuto {

	public final Pose2d initialPose = new Pose2d( -36, 63.5, Math.toRadians(-90));
	@Override
	public Command setupAuto(CommandScheduler scheduler) {

		Trajectory goNearScoring1 = robot.drivetrain.getBuilder().trajectoryBuilder(initialPose)
			.lineToLinearHeading(new Pose2d(-36.0, 18, Math.toRadians(-90)))
				.build();

		Trajectory placeCone = robot.drivetrain.getBuilder().trajectoryBuilder(goNearScoring1.end())
				.lineToLinearHeading(new Pose2d(-30, 9, Math.toRadians(-75)))
				.build();


		Trajectory goToPickupPartial = robot.drivetrain.getBuilder().trajectoryBuilder(placeCone.end(),true)
				.splineToLinearHeading(new Pose2d(-48,13,Math.toRadians(0)),Math.toRadians(0))
				.build();


		return follow(goNearScoring1)
				.addNext(new MultipleCommand(new GoToScore(robot.scoringMechanism, ScoringMechanism.States.HIGH), follow(placeCone)))
				.addNext(new AlignWithVision2Auto(robot.drivetrain, robot.distanceSensor))
				.addNext(new DepositAuto(robot.scoringMechanism))
				.addNext(new Delay(2))
				.addNext(follow(goToPickupPartial))
				.addNext(new ActivateIntakeAuto(robot.scoringMechanism));


	}

	@Override
	public void setRobotPosition() {
		robot.drivetrain.setPose(initialPose);
	}
}
