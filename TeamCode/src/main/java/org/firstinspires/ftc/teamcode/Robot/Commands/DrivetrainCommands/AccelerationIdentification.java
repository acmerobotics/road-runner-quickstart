package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Odometry;

public class AccelerationIdentification extends Command {
    Drivetrain drivetrain;
    Odometry odometry;

    ElapsedTime timer = new ElapsedTime();
    double maxLength;
    double kV;
    double kS;
    double power;

    double velocityLeftPrev = 0;
    double velocityRightPrev = 0;
    double timePrev = 0;

    public AccelerationIdentification(Drivetrain drivetrain, Odometry odometry, double kV, double kS, double power, double maxLength) {
        super(drivetrain, odometry);

        this.drivetrain = drivetrain;
        this.odometry = odometry;
        this.kV = kV;
        this.kS = kS;
        this.power = power;
        this.maxLength = maxLength;
    }


    @Override
    public void init() {
        timer.reset();
        drivetrain.setPower(power,power);
    }

    @Override
    public void periodic() {
        double time = timer.seconds();
        double velocityLeft = Odometry.encoderTicksToInches(odometry.leftEncoder.getVelocity());
        double velocityRight = Odometry.encoderTicksToInches(odometry.rightEncoder.getVelocity());

        double accelerationLeft = (velocityLeft - velocityLeftPrev) / (time - timePrev);
        double accelerationRight = (velocityRight - velocityRightPrev) / (time - timePrev);

        double leftAccelerationPower = drivetrain.getLeftPower() - (kV * velocityLeft + kS);
        double rightAccelerationPower = drivetrain.getRightPower() - (kV * velocityRight + kS);

        Dashboard.packet.put("Velocity", Odometry.encoderTicksToInches(odometry.leftEncoder.getVelocity()));
        Dashboard.packet.put("Acceleration", accelerationLeft);
        Dashboard.packet.put("Velocity Delta", velocityLeft - velocityLeftPrev);

        RobotLog.ii("SysID (P/V)^2", leftAccelerationPower + ":" + accelerationLeft + "," + rightAccelerationPower + ":" + accelerationRight);

        velocityLeftPrev = velocityLeft;
        velocityRightPrev = velocityRight;
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
