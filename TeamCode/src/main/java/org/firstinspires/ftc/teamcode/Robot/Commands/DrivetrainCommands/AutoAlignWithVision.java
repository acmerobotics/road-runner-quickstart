package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.PoleDetectionSubsystem;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.visionPiplines.Pole;

//detected Pole: Pole{xPixel=363.0, width=138.0, isValidPole=true}
public class AutoAlignWithVision extends Command {

	Drivetrain drivetrain;
	PoleDetectionSubsystem poleVision;

	final double horizontalTolerance = 3; // pixel tolerance left and right for exit
	final double widthTolerance = 10; // pixel tolerance of forward back pole width

	PIDCoefficients horizontalCoefficients = new PIDCoefficients(0.01,0,0);
	PIDCoefficients forwardCoefficients = new PIDCoefficients(0.01,0,0);

	protected BasicPID horizontalPID = new BasicPID(horizontalCoefficients);
	protected BasicPID forwardBackPID = new BasicPID(forwardCoefficients);
	double horizontalReference = 363.0;
	double widthReference = 138.0;
	Pole pole = new Pole(0,0);


	public AutoAlignWithVision(Drivetrain drivetrain, PoleDetectionSubsystem poleVision) {
		super(drivetrain,poleVision);
		this.drivetrain = drivetrain;
		this.poleVision = poleVision;
	}

	@Override
	public void init() {

	}

	@Override
	public void periodic() {
		pole = poleVision.pipeline.getPole();

		double forward = forwardBackPID.calculate(widthReference,pole.width);
		double side = -horizontalPID.calculate(horizontalReference,pole.xPixel);
		drivetrain.fieldRelative(new Pose2d(forward,side,0));

	}

	@Override
	public boolean completed() {

		if (!poleVision.pipeline.getPole().isValidPole) {
			return true;
		}
		return Math.abs(horizontalReference - pole.xPixel) < horizontalTolerance && Math.abs(widthReference - pole.width) < widthReference;
	}

	@Override
	public void shutdown() {
		drivetrain.robotRelative(new Pose2d());
	}
}
