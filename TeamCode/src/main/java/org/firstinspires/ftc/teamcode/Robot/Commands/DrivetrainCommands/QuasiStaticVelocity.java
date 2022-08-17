package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Odometry;

public class QuasiStaticVelocity extends Command {
    Drivetrain drivetrain;
    Odometry odometry;
    ElapsedTime timer = new ElapsedTime();

    double rampRate;
    double maxLength;

    public QuasiStaticVelocity(Drivetrain drivetrain, Odometry odometry, double rampRate, double maxLength) {
        super(drivetrain, odometry);

        this.drivetrain = drivetrain;
        this.odometry = odometry;
        this.rampRate = rampRate;
        this.maxLength = maxLength;
    }


    @Override
    public void init() {
        timer.reset();
        drivetrain.setPower(0,0);
    }

    @Override
    public void periodic() {
        double leftPower = Range.clip(drivetrain.getLeftPower(), 0.00, 1);
        double leftVelocity = Odometry.encoderTicksToInches(odometry.leftEncoder.getVelocity());

        double rightPower = Range.clip(drivetrain.getRightPower(), 0.00, 1);
        double rightVelocity = Odometry.encoderTicksToInches(odometry.rightEncoder.getVelocity());

        RobotLog.ii("SysID (P/V)", "(" + leftPower + "," + leftVelocity + "),(" + rightPower + "," + rightVelocity + ")");

        double newPower = timer.seconds() * rampRate / 12;
        drivetrain.setPower(newPower, newPower);
    }

    @Override
    public boolean completed() {
        return timer.seconds() > maxLength;
    }

    @Override
    public void shutdown() {
        drivetrain.setPower(0,0);
    }
}
