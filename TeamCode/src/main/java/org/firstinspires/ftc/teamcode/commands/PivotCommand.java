package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;

import java.util.function.DoubleSupplier;

public class PivotCommand extends CommandBase {
    private final PivotSubsystem pivotSubsystem;
//    private final DoubleSupplier power;
    private final double angle;

    public PivotCommand(final PivotSubsystem pivot, final double angle) {
        pivotSubsystem = pivot;
//        this.power = power;
        this.angle = angle;

        addRequirements(pivotSubsystem);
    }

    @Override
    public void execute() {
        pivotSubsystem.setAngle(angle);
    }

}
