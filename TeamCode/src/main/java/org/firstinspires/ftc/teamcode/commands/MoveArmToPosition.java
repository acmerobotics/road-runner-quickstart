package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.ExampleArmSubsystem;

public class MoveArmToPosition extends CommandBase {

    private final ExampleArmSubsystem m_exampleSubsystem;
    private final int position;

    /**
     * Sets the dual-motor arm to the specified position.
     * @param exampleArmSubsystem Parent subsystem - ExampleArmSubsystem by default
     * @param position            Takes a position to set the motor to using PID
     *
     * @see org.firstinspires.ftc.teamcode.Constants
     * @see DcMotor#setTargetPosition(int)
     */
    public MoveArmToPosition(ExampleArmSubsystem exampleArmSubsystem,
                             int position) {
        m_exampleSubsystem = exampleArmSubsystem;
        this.position = position;
        addRequirements();
    }

    @Override
    public void initialize(){
        m_exampleSubsystem.moveArmToPosition(position);
    }

    @Override
    public void execute(){

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
