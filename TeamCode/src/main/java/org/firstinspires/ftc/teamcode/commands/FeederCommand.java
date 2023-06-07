package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;

import java.util.function.DoubleSupplier;

public class FeederCommand extends CommandBase {
    private final FeederSubsystem feederSubsystem;
    private final DoubleSupplier power;

    public FeederCommand(final FeederSubsystem feeder, final DoubleSupplier power) {
        feederSubsystem = feeder;
        this.power = power;

        addRequirements(feederSubsystem);
    }

    @Override
    public void execute() {
        feederSubsystem.setPower(power.getAsDouble());
    }

}
