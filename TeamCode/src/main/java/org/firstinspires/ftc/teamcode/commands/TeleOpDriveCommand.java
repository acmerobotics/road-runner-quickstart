package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class TeleOpDriveCommand extends CommandBase {
    private final DriveSubsystem drive;

    private final DoubleSupplier x;
    private final DoubleSupplier y;
    private final DoubleSupplier omega;

    public TeleOpDriveCommand(final DriveSubsystem drive, final DoubleSupplier y, final DoubleSupplier x, final DoubleSupplier omega) {
        this.drive = drive;

        this.y = y;
        this.x = x;
        this.omega = omega;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.setWeightedDrivePower(new Pose2d(-y.getAsDouble(), x.getAsDouble(), omega.getAsDouble()));
    }
}
