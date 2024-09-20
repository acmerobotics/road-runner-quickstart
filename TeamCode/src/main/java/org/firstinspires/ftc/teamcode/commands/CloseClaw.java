package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ExampleArmSubsystem;

public class CloseClaw extends CommandBase {

    private final ExampleArmSubsystem m_exampleArmSubsystem;

    /**
     * Command to make the claw close.
     *
     * @param exampleArmSubsystem Parent subsystem - ExampleArmSubsystem by default
     *
     * @see ExampleArmSubsystem#closeClaw()
     */
    public CloseClaw(ExampleArmSubsystem exampleArmSubsystem){
        this.m_exampleArmSubsystem = exampleArmSubsystem;
        addRequirements();
    }

    @Override
    public void initialize(){
        m_exampleArmSubsystem.closeClaw();
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
