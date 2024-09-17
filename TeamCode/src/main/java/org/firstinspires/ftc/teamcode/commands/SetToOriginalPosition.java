package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ExampleArmSubsystem;

public class SetToOriginalPosition extends CommandBase {

    private final ExampleArmSubsystem m_exampleArmSubsystem;

    /**
     * Command to set the arm to the original position specified in <code>Constants.java</code>
     *
     * @param exampleArmSubsystem Parent subsystem - ExampleArmSubsystem by default
     *
     * @see org.firstinspires.ftc.teamcode.Constants
     */
    public SetToOriginalPosition(ExampleArmSubsystem exampleArmSubsystem){
        this.m_exampleArmSubsystem = exampleArmSubsystem;
        addRequirements();
    }

    @Override
    public void initialize(){
        m_exampleArmSubsystem.resetToOriginalPosition();
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
