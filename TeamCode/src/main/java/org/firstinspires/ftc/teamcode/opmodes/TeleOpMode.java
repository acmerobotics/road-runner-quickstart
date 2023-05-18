package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.FeederCommand;
import org.firstinspires.ftc.teamcode.commands.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;

@Config
@TeleOp(group="TeleOp")
public class TeleOpMode extends CommandOpMode {
    private DriveSubsystem driveSubsystem;
    private PivotSubsystem pivotSubsystem;
    private FeederSubsystem feederSubsystem;

    private TeleOpDriveCommand driveCommand;
    private PivotCommand pivotCommand;
    private FeederCommand feederCommand;
    private GamepadEx driverController;

    @Override
    public void initialize() {
        this.driverController = new GamepadEx(gamepad1);
        this.driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        this.pivotSubsystem = new PivotSubsystem(hardwareMap, telemetry);
        this.feederSubsystem = new FeederSubsystem(hardwareMap);

        this.driveCommand = new TeleOpDriveCommand(driveSubsystem, driverController::getLeftY, driverController::getLeftX, driverController::getRightX);
        this.pivotCommand = new PivotCommand(pivotSubsystem, () -> (driverController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - driverController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));
        this.feederCommand = new FeederCommand(feederSubsystem, (driverController.getButton(GamepadKeys.Button.A)) ? -0.5 : 0.5);

        register(driveSubsystem);
        register(pivotSubsystem);
        register(feederSubsystem);

        setDefaultCommands();
    }

    private void setDefaultCommands() {
        driveSubsystem.setDefaultCommand(driveCommand);
        pivotSubsystem.setDefaultCommand(pivotCommand);
        feederSubsystem.setDefaultCommand(feederCommand);
    }


}
