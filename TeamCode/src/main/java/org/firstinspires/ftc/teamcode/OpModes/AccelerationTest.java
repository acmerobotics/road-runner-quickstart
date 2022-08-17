package org.firstinspires.ftc.teamcode.OpModes;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.ThermalEquilibrium.homeostasis.Utils.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.AccelerationIdentification;

@Autonomous
public class AccelerationTest extends BaseAuto {
    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public Command setupAuto(CommandScheduler scheduler) {
        robot.odometry.setEstimate(new Vector(new double[]{0, 0, 0}));

        AccelerationIdentification accelerationTest = new AccelerationIdentification(robot.drivetrain, robot.odometry, 0.011017662373924956, 0.043117850304622056, 0.7, 30);

        return accelerationTest;
    }
}
