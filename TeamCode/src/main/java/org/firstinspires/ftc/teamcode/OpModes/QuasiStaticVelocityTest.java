package org.firstinspires.ftc.teamcode.OpModes;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.ThermalEquilibrium.homeostasis.Utils.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.QuasiStaticVelocity;

@Autonomous
public class QuasiStaticVelocityTest extends BaseAuto {
    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public Command setupAuto(CommandScheduler scheduler) {
        robot.odometry.setEstimate(new Vector(new double[]{0, 0, 0}));

        QuasiStaticVelocity quasiStaticTest = new QuasiStaticVelocity(robot.drivetrain, robot.odometry, 0.25, 30);

        return quasiStaticTest;
    }
}
