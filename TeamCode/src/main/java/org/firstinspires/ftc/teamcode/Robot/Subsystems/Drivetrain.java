package org.firstinspires.ftc.teamcode.Robot.Subsystems;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.RR_quickstart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;

public class Drivetrain extends Subsystem {
	protected HardwareMap hwMap;

	protected double leftPower = 0;
	protected double rightPower = 0;
	SampleMecanumDrive drive;




	@Override
	public void initAuto(HardwareMap hwMap) {
		this.hwMap = hwMap;
		drive = new SampleMecanumDrive(hwMap);

	}
	@Override
	public void periodic() {
		drive.update();
	}


	public void  robotRelative(Pose2d powers) {
		drive.setWeightedDrivePower(powers);
	}

	public void fieldRelative(Pose2d powers) {
		Vector2d vec = new Vector2d(powers.getX(),-powers.getY());

		vec = vec.rotated(drive.getPoseEstimate().getHeading());
		powers = new Pose2d(vec.getX(),-vec.getY(),powers.getHeading());
		robotRelative(powers);
	}


	public void followTrajectory(Trajectory traj) {
		drive.followTrajectoryAsync(traj);
	}

	@Override
	public void shutdown() {
		drive.setMotorPowers(0,0,0,0);
	}

	public double getLeftPower() {
		return leftPower;
	}

	public double getRightPower() {
		return rightPower;
	}

	public Pose2d getPose() {
		return drive.getPoseEstimate();
	}

	public boolean isBusy() {
		return drive.isBusy();
	}

	public SampleMecanumDrive getBuilder() {
		return drive;
	}

	public Pose2d getVelocity() {
		return drive.getPoseVelocity();
	}

	public void setPose(Pose2d pose) {
		drive.setPoseEstimate(pose);
	}
}
