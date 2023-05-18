package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Config
@TeleOp(group="teleop")
public class TeleOpMode extends CommandOpMode {
    private DriveSubsystem driveSubsystem;
    private TeleOpDriveCommand driveCommand;

    private GamepadEx driverController;

    @Override
    public void initialize() {
        this.driverController = new GamepadEx(gamepad1);
        this.driveSubsystem = new DriveSubsystem(hardwareMap);
        this.driveCommand = new TeleOpDriveCommand(driveSubsystem, driverController::getLeftX, driverController::getLeftY, driverController::getRightX);

        register(driveSubsystem);

        setDefaultCommands();
    }

    private void setDefaultCommands() {
        driveSubsystem.setDefaultCommand(driveCommand);
    }
}
