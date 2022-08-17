package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;



import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Purepursuit.CurveCalculator;
import org.firstinspires.ftc.teamcode.Purepursuit.CurvePoint;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Robot;

import java.util.ArrayList;

public class DrivePurePursuit extends Command {




    Robot robot;
    ArrayList<CurvePoint> points;
    CurveCalculator PurePursuit;

    public DrivePurePursuit(Robot robot, ArrayList<CurvePoint> points) {
        this.robot = robot;
        this.points = points;
        this.PurePursuit = new CurveCalculator();
    }


    @Override
    public void init() {

    }

    @Override
    public void periodic() {
        double[] driveSignal = PurePursuit.getDriveSignal(points, robot.odometry.getPose());
        robot.drivetrain.robotRelative(driveSignal[0],driveSignal[1]);

    }

    @Override
    public boolean completed() {
        return PurePursuit.isDone();
    }

    @Override
    public void shutdown() {

    }
}
