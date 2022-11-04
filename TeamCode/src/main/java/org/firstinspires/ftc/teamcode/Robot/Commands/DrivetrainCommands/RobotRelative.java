package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;


import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Input;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Robot;

public class RobotRelative extends Command {


    Drivetrain drivetrain;
    Input game_pad1;

    double strafe_dead_band = 0.1;

    public RobotRelative(Robot robot, Input game_pad1) {
        super(robot.drivetrain, game_pad1);
        this.drivetrain = robot.drivetrain;
        this.game_pad1 = game_pad1;

    }

    @Override
    public void init() {

    }

    @Override
    public void periodic() {
        double x;
        double y;
        double turn;
        x = game_pad1.getForwardJoystick();
        y = game_pad1.getStrafeJoystick();
        turn = game_pad1.getTurnJoystick();
        System.out.println("forward: " + x + " strafe: " + y + " turn: " + turn);


        if (Math.abs(y) < strafe_dead_band) {
            y = 0;
        }

        Pose2d powers = new Pose2d(x,y,turn);
        drivetrain.fieldRelative(powers);


    }

    @Override
    public boolean completed() {
        return false;
    }

    @Override
    public void shutdown() {
        drivetrain.shutdown();
    }
}
