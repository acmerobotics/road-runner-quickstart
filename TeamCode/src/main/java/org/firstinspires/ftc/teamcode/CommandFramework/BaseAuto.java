package org.firstinspires.ftc.teamcode.CommandFramework;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.DriveDistance;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.TurnCommand;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Robot;

public abstract class BaseAuto extends LinearOpMode {

	protected Robot robot;

	@Override
	public void runOpMode() {
		robot = new Robot(hardwareMap, Robot.OpMode.Auto, gamepad1, gamepad2);
		waitForStart();

		robot.getScheduler().forceCommand(setupAuto(robot.getScheduler()));
		ElapsedTime timer = new ElapsedTime();

		double prevHeading = 0;
		while (opModeIsActive()) {
			double odomHeading = robot.odometry.getPose().getHeading();
			double headingDelta = odomHeading - prevHeading;
			prevHeading = odomHeading;
			System.out.println("ML FINAL DATA: " + robot.drivetrain.getLeftPower() +
					", " + robot.drivetrain.getRightPower() + ", " + odomHeading + ", "
			+ robot.odometry.getPose().getX() + ", " + robot.odometry.getPose().getY() + ", " +
					robot.odometry.getLeftVelocity() + ", " + robot.odometry.getRightVelocity() + ", " + timer.seconds()
			+ ", " + robot.odometry.getLeftDelta() + ", " + robot.odometry.getRightDelta() + ", " + headingDelta);
			timer.reset();
			robot.update();

		}

	}

	public abstract Command setupAuto(CommandScheduler scheduler);


	public DriveDistance drive(double distance) {
		return new DriveDistance(robot.drivetrain, robot.odometry, distance);
	}
	@RequiresApi(api = Build.VERSION_CODES.N)
	public TurnCommand turn(double radians) {
		return new TurnCommand(robot.drivetrain, robot.odometry, radians);
	}
}
