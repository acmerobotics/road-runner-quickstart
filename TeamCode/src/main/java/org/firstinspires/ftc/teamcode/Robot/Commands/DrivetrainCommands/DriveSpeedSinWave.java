package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Math.Controllers.MotorModelController;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Odometry;

public class DriveSpeedSinWave extends Command {
    Drivetrain drivetrain;
    Odometry odom;

    ElapsedTime timer = new ElapsedTime();

    double seconds;
    double speed;
    double maxAccel;

    MotorModelController leftController = new MotorModelController(0.010752544908791628, 0.038496093679464466, 0.04383880611593726);
    MotorModelController rightController = new MotorModelController(0.011017662373924956,0.04229443721708967 , 0.043117850304622056);

    public DriveSpeedSinWave(Drivetrain drivetrain, Odometry odom, double seconds, double speed, double maxAccel) {
        super(drivetrain, odom);
        this.drivetrain = drivetrain;
        this.odom = odom;

        this.seconds = seconds;
        this.speed = speed;
        this.maxAccel = maxAccel;
    }


    @Override
    public void init() {
        timer.reset();
        drivetrain.robotRelative(0,0);
    }

    @Override
    public void periodic() {
        double speed = this.speed + Math.sin(timer.seconds() * 3) * 5;

        double leftVelocity = Odometry.encoderTicksToInches(odom.leftEncoder.getVelocity());
        double rightVelocity = Odometry.encoderTicksToInches(odom.rightEncoder.getVelocity());

        drivetrain.setPower(leftController.calculate(speed, leftVelocity, maxAccel), rightController.calculate(speed, rightVelocity, maxAccel));

        Dashboard.packet.put("Left velocity", leftVelocity);
        Dashboard.packet.put("Right velocity", rightVelocity);
        Dashboard.packet.put("Target velocity", speed);
    }

    @Override
    public boolean completed() {
        return timer.seconds() > seconds;
    }

    @Override
    public void shutdown() {
        drivetrain.setPower(0,0);
    }
}
