package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;

@Autonomous
public class BlueRight extends BaseAuto {

	public final Pose2d initialPose = new Pose2d( -36, 63.5, -Math.toRadians(90));
	public final Pose2d pickupLocation = new Pose2d(-60, 12, Math.toRadians(-90));
	@Override
	public Command setupAuto(CommandScheduler scheduler) {

//
//		Trajectory traj1 = robot.drivetrain.getBuilder().trajectoryBuilder(initialPose)
//			.lineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(-180)))
//				.build();

		Trajectory traj1 = robot.drivetrain.getBuilder().trajectoryBuilder(initialPose)
				.splineToSplineHeading(pickupLocation, Math.toRadians(-180))
				.build();



		return follow(traj1);
	}

	@Override
	public void setRobotPosition() {
		robot.drivetrain.setPose(initialPose);
	}
}
