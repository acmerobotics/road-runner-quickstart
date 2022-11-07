package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;

import android.os.Build;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DistanceSensor;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;

import java.util.function.BooleanSupplier;

public class AlignWithVision2Auto extends Command {
	Drivetrain drivetrain;
	DistanceSensor distanceSensor;

	public static PIDCoefficients controllerCoefficientsDistance = new PIDCoefficients(0.1,0,0.05);
	protected BasicPID controller = new BasicPID(controllerCoefficientsDistance);
	public static double referenceDistanceSensor = 12; // distance in inches away from the pole

	double error = 10;
	double error_tolerance = 0.5;


	public AlignWithVision2Auto(Drivetrain drivetrain, DistanceSensor distanceSensor) {
		this.drivetrain = drivetrain;
		this.distanceSensor = distanceSensor;
	}

	@Override
	public void init() {

	}

	@Override
	public void periodic() {

		double power = controller.calculate(referenceDistanceSensor, distanceSensor.getDistance_in());
		drivetrain.robotRelative(new Pose2d(-power,0,0));
		error = referenceDistanceSensor - distanceSensor.getDistance_in();
	}

	@Override
	public boolean completed() {
		return Math.abs(error) < error_tolerance;
	}

	@Override
	public void shutdown() {
		drivetrain.robotRelative(new Pose2d(0,0,0));
	}

}
