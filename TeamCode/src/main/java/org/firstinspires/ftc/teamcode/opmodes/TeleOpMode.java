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
    
    private GamepadButton zeroPos;
    private GamepadButton feedPos;
    private GamedpadButton scorePos;
    

    @Override
    public void initialize() {
        this.driverController = new GamepadEx(gamepad1);
        this.driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        this.pivotSubsystem = new PivotSubsystem(hardwareMap, telemetry);
        this.feederSubsystem = new FeederSubsystem(hardwareMap);
        
        this.zeroPos = new GamepadButton(driverController, GamepadKeys.Button.DPAD_UP);
        this.feedPos = new GamepadButton(driverController, GamepadKeys.Button.DPAD_UP);
        this.scorePos = new GamepadButton(driverController, GamepadKeys.Button.DPAD_UP);

        this.driveCommand = new TeleOpDriveCommand(driveSubsystem, driverController::getLeftY, driverController::getLeftX, driverController::getRightX);
//        this.pivotCommand = new PivotCommand(pivotSubsystem, () -> (driverController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - driverController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));
        this.feederCommand = new FeederCommand(feederSubsystem, (driverController.getButton(GamepadKeys.Button.RIGHT_BUMPER)) ? -0.5 : 0.3);

        register(driveSubsystem);
        register(pivotSubsystem);
        register(feederSubsystem);

        setDefaultCommands();
    }

  
    private void configureButtonBindings(){
        zeroPos.whenPressed(pivotCommand(pivotSubsystem, 0.0));
        feedPos.whenPressed(pivotCommand(pivotSubsystem, 0.0));
        scorePos.whenPressed(pivotCommand(pivotSubsystem, 0.0));
    }
    

    private void setDefaultCommands() {
        driveSubsystem.setDefaultCommand(driveCommand);
  //      pivotSubsystem.setDefaultCommand(pivotCommand);
        feederSubsystem.setDefaultCommand(feederCommand);
    }


}
