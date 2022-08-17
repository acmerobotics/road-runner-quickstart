package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Input;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Odometry;

import static org.firstinspires.ftc.teamcode.Robot.ControlConstants.AngularVelocityTeleop;
import static org.firstinspires.ftc.teamcode.Robot.ControlConstants.AngularVelocityTeleopFF;

public class ClosedLoopTeleop extends Command {

	public final double MAX_ANGULAR_VELOCITY = 8; // radians / s
	BasicPID control = new BasicPID(AngularVelocityTeleop);
	BasicFeedforward feedforward = new BasicFeedforward(AngularVelocityTeleopFF);
	Drivetrain drivetrain;
	Odometry odom;
	Input gamepad;

	public ClosedLoopTeleop(Drivetrain drivetrain,Odometry odom,Input gamepad) {
		super(drivetrain,odom,gamepad);
		this.drivetrain = drivetrain;
		this.odom = odom;
		this.gamepad = gamepad;
	}

	@Override
	public void init() {

	}

	@Override
	public void periodic() {

		double targetVelocity = gamepad.getRight_stick_x() * MAX_ANGULAR_VELOCITY;
		double currentVelocity = odom.getVelocity().get(2);

		Dashboard.packet.put("target velocity", targetVelocity);
		Dashboard.packet.put("other gyro velocity", currentVelocity);

 		double feedforwardCommand = feedforward.calculate(0,targetVelocity,0);
		double feedbackCommand = control.calculate(targetVelocity, currentVelocity);
		double forward = -gamepad.getLeft_stick_y();

		drivetrain.robotRelative(forward, feedforwardCommand + feedbackCommand);

	}

	@Override
	public boolean completed() {
		return false;
	}

	@Override
	public void shutdown() {

	}

}
