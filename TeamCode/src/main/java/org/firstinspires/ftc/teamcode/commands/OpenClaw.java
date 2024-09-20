package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ExampleArmSubsystem;

public class OpenClaw extends CommandBase {

    private final ExampleArmSubsystem m_exampleArmSubsystem;

    /**
     * Command to make the claw open.
     *
     * @param exampleArmSubsystem Parent subsystem - ExampleArmSubsystem by default
     *
     * @see ExampleArmSubsystem#openClaw()
     */
    public OpenClaw(ExampleArmSubsystem exampleArmSubsystem){
        this.m_exampleArmSubsystem = exampleArmSubsystem;
        addRequirements();
    }

    @Override
    public void initialize(){
        m_exampleArmSubsystem.openClaw();
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
