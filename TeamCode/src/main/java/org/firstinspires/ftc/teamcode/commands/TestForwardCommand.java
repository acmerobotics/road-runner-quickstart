package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TestSubsystem;

public class TestForwardCommand extends CommandBase {

    private final TestSubsystem m_testSubsystem;

    public TestForwardCommand(TestSubsystem testSubsystem) {
        this.m_testSubsystem = testSubsystem;
        addRequirements(m_testSubsystem);
    }

    @Override
    public void initialize(){
        m_testSubsystem.forward();
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
