package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Robot;

public class RoadrunnerTrajectoryFollower extends Command {


    private final Robot robot;
    Trajectory traj;


    public RoadrunnerTrajectoryFollower(Robot robot, Trajectory traj) {
        super(robot.drivetrain);
        this.robot = robot;
        this.traj = traj;
    }


    @Override
    public void init() {
        robot.drivetrain.followTrajectory(traj);
    }

    @Override
    public void periodic() {

    }

    @Override
    public boolean completed() {
        return !robot.drivetrain.isBusy();
    }

    @Override
    public void shutdown() {

    }
}
