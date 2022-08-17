package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.ThermalEquilibrium.homeostasis.Utils.Vector;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.Math.Controllers.DistanceDriveControl;
import org.firstinspires.ftc.teamcode.Utils.ExtraUtils;

public class DriveDistance extends Command {

	DistanceDriveControl control;
	Drivetrain drivetrain;
	Odometry odom;

	double targetDistance;

	Vector initialPosition;
	private boolean isComplete = false;

	public DriveDistance(Drivetrain drivetrain, Odometry odom, double targetDistance) {
		super(drivetrain, odom);

		this.drivetrain = drivetrain;
		this.odom = odom;
		this.targetDistance = targetDistance;
	}


	@RequiresApi(api = Build.VERSION_CODES.N)
	@Override
	public void init() {
		initialPosition = odom.getPosition();
		double initialAngle = odom.getPosition().get(2);

		control = new DistanceDriveControl(() -> odom.getPosition().get(2), initialAngle);

	}

	@RequiresApi(api = Build.VERSION_CODES.N)
	@Override
	public void periodic() {
		double traveled = ExtraUtils.calculateDistance(initialPosition,
				odom.getPosition());
		drivetrain.setPower(control.calculate(targetDistance,traveled));
		isComplete = Math.abs(control.endPoseError()) < 1 && Math.abs(odom.getVelocity().get(0)) < 3;

	}

	@Override
	public boolean completed() {
		return isComplete;
	}

	@Override
	public void shutdown() {
		drivetrain.setPower(0,0);
	}
}
