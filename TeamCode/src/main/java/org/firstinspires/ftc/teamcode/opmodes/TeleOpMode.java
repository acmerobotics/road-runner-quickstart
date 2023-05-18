package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;

@Config
@TeleOp(group="TeleOp")
public class TeleOpMode extends CommandOpMode {
    private DriveSubsystem driveSubsystem;
    private PivotSubsystem pivotSubsystem;

    private TeleOpDriveCommand driveCommand;
    private PivotCommand pivotCommand;

    private GamepadEx driverController;

    @Override
    public void initialize() {
        this.driverController = new GamepadEx(gamepad1);
        this.driveSubsystem = new DriveSubsystem(hardwareMap);
        this.pivotSubsystem = new PivotSubsystem(hardwareMap);

        this.driveCommand = new TeleOpDriveCommand(driveSubsystem, driverController::getLeftY, driverController::getLeftX, driverController::getRightX);
        this.pivotCommand = new PivotCommand(pivotSubsystem, () -> (driverController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - driverController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));

        register(driveSubsystem);
        register(pivotSubsystem);

        setDefaultCommands();
    }

    private void setDefaultCommands() {
        driveSubsystem.setDefaultCommand(driveCommand);
        pivotSubsystem.setDefaultCommand(pivotCommand);
    }
}
