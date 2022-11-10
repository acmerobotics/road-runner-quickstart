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

		Pose2d goNearScoring = new Pose2d(-36.0, 18, Math.toRadians(-90));
		Pose2d placeCone = new Pose2d(-29, 16.5, Math.toRadians(-75));
		Pose2d pickupPartial = new Pose2d(-48,14.5,Math.toRadians(0));
		Pose2d pickupFull = new Pose2d(-60,14.5,Math.toRadians(0));

		return goToLQR(goNearScoring)
				.addNext(new MultipleCommand(new GoToScore(robot.scoringMechanism, ScoringMechanism.States.HIGH), goToLQR(placeCone)))
				.addNext(new DepositAuto(robot.scoringMechanism))
				.addNext(new MultipleCommand(new Delay(2), goToLQR(pickupPartial)))
				.addNext(goToLQR(pickupFull))
				.addNext(new ActivateIntakeAuto(robot.scoringMechanism))
				.addNext(goToLQR(goNearScoring))
				.addNext(new MultipleCommand(new GoToScore(robot.scoringMechanism, ScoringMechanism.States.HIGH), goToLQR(placeCone)))
				.addNext(new DepositAuto(robot.scoringMechanism))
				.addNext(new Delay(2));


	}

	@Override
	public void setRobotPosition() {
		robot.drivetrain.setPose(initialPose);
	}
}
