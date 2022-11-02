package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.PoleDetectionSubsystem;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Utils.LinearInterpolator;
import org.firstinspires.ftc.teamcode.visionPiplines.Pole;

import java.util.ArrayList;


public class AutoAlignWithVision extends Command {

	Drivetrain drivetrain;
	PoleDetectionSubsystem poleVision;

	final double horizontalTolerance = 50; // pixel tolerance left and right for exit
	final double widthTolerance = 50; // pixel tolerance of forward back pole width

	PIDCoefficients horizontalCoefficients = new PIDCoefficients(0.005,0,0);
	PIDCoefficients forwardCoefficients = new PIDCoefficients(0,0,0);

	protected BasicPID horizontalPID = new BasicPID(horizontalCoefficients);
	protected BasicPID forwardBackPID = new BasicPID(forwardCoefficients);
	double horizontalReference = 363.0;
	double widthReference = 138.0;
	Pole pole = new Pole(0,0);
	LinearInterpolator interpolatorSide;
	LinearInterpolator interpolatorForward;
	LinearInterpolator angleInterpolator;


	public AutoAlignWithVision(Drivetrain drivetrain, PoleDetectionSubsystem poleVision) {
		super(drivetrain,poleVision);
		this.drivetrain = drivetrain;
		this.poleVision = poleVision;

		double [] xs = {0,2.5,-1.8,-1.8024541968487422,-1.0340386610282355,2.7879328083329105};
		ArrayList<Double> Positions = new ArrayList<>();
		for (double d: xs) {
			Positions.add(d);
		}

		double [] x2s = {352, 92.0,562.0, 555.0, 434.0, 89.0};
		ArrayList<Double> xPixels = new ArrayList<>();
		for (double d: x2s) {
			xPixels.add(d);
		}
		interpolatorSide = new LinearInterpolator(xPixels,Positions);

		double [] forwards = {0,-0.1,-0.4,1.0793615056637846,-5.5,-0.17};
		double [] widths = {109.0,117.0,113.0, 129.0,73.0,107.0};

		ArrayList<Double> forwardPositions = new ArrayList<>();
		ArrayList<Double> widthPixels = new ArrayList<>();
		for (double d: forwards) {
			forwardPositions.add(d);
		}
		for (double d: widths) {
			widthPixels.add(d);
		}
		interpolatorForward = new LinearInterpolator(widthPixels,forwardPositions);

		double [] angles = {0,5.7375449833905705,9.037768746923982,360-357.5293737696849,360-349.59975355253454};
		double [] polePixels2 = {118,166,73,370,570};
		ArrayList<Double> anglesToPole = new ArrayList<>();
		ArrayList<Double> polePositionForAngle = new ArrayList<>();
		for (double d: angles) {
			anglesToPole.add(d);
		}
		for (double d: polePixels2) {
			polePositionForAngle.add(d);
		}
		angleInterpolator = new LinearInterpolator(polePositionForAngle,anglesToPole);

	}

	@Override
	public void init() {
		pole = poleVision.pipeline.getPole();

		if (pole.isValidPole) {
			Pose2d relativePose = new Pose2d(interpolatorForward.getValue(pole.width), interpolatorSide.getValue(pole.xPixel), Math.toRadians(angleInterpolator.getValue(pole.xPixel)));
			drivetrain.setPose(relativePose);
			Trajectory traj = drivetrain.getBuilder().trajectoryBuilder(relativePose)
					.lineToLinearHeading(new Pose2d(-0.5,-2,0))
					.build();

			drivetrain.followTrajectory(traj);
		}




	}

	@Override
	public void periodic() {



	}

	@Override
	public boolean completed() {

		if (!poleVision.pipeline.getPole().isValidPole) {
			return true;
		}
		return !drivetrain.isBusy();
	}

	@Override
	public void shutdown() {
		drivetrain.robotRelative(new Pose2d());
	}
}
