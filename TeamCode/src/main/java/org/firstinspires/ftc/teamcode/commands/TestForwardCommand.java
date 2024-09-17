package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TestSubsystem;

public class TestForwardCommand extends CommandBase {

    private final TestSubsystem m_testSubsystem;

    /**
     * Command to set the test motor to run forwards at specified speed in <code>Constants.java</code>.
     *
     * @param testSubsystem Parent subsystem - Currently TestSubsystem
     *
     * @see org.firstinspires.ftc.teamcode.Constants
     */
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
