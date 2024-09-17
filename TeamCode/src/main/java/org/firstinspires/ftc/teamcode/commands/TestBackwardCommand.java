package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TestSubsystem;

public class TestBackwardCommand extends CommandBase {

    private final TestSubsystem m_testSubsystem;

    /**
     * Command to set the test motor to run backwards at specified speed in <code>Constants.java</code>.
     *
     * @param testSubsystem Parent subsystem - Currently TestSubsystem
     *
     * @see org.firstinspires.ftc.teamcode.Constants
     */
    public TestBackwardCommand(TestSubsystem testSubsystem) {
        this.m_testSubsystem = testSubsystem;
        addRequirements(m_testSubsystem);
    }

    @Override
    public void initialize(){
        m_testSubsystem.backward();
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
