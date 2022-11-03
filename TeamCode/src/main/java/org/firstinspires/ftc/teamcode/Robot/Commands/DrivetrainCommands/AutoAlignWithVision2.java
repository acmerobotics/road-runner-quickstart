package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;

import android.os.Build;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DistanceSensor;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Input;

import java.util.function.BooleanSupplier;

public class AutoAlignWithVision2 extends Command {

	Drivetrain drivetrain;
	DistanceSensor distanceSensor;

	protected PIDCoefficients controllerCoefficients = new PIDCoefficients(0.1,0,0.05);
	protected BasicPID controller = new BasicPID(controllerCoefficients);
	BooleanSupplier keepRunning;
	double referenceDistance = 10; // distance in inches away from the pole


	public AutoAlignWithVision2(Drivetrain drivetrain, DistanceSensor distanceSensor, BooleanSupplier keepRunning) {
		this.drivetrain = drivetrain;
		this.distanceSensor = distanceSensor;
		this.keepRunning = keepRunning;
	}

	@Override
	public void init() {

	}

	@Override
	public void periodic() {

		double power = controller.calculate(referenceDistance, distanceSensor.getDistance_in());
		drivetrain.robotRelative(new Pose2d(-power,0,0));
	}

	@Override
	public boolean completed() {
		if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
			return !keepRunning.getAsBoolean();
		}
		return true;
	}

	@Override
	public void shutdown() {
		drivetrain.robotRelative(new Pose2d(0,0,0));
	}
}
