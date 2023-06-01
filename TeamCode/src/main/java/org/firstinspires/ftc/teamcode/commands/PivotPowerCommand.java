package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;

import java.util.function.DoubleSupplier;

@Config
public class PivotPowerCommand extends CommandBase {
    private final PivotSubsystem pivot;
    private final DoubleSupplier power;
    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public static double speed = 0.25;

    public PivotPowerCommand(PivotSubsystem pivot, DoubleSupplier power) {
        this.pivot = pivot;
        this.power = power;

        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        timer.reset();
        pivot.setAngle(pivot.getAngle() + (power.getAsDouble() * timer.time() * speed));
    }
}
