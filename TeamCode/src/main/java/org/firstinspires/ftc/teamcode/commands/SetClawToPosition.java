package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.checkerframework.checker.units.qual.degrees;
import org.firstinspires.ftc.teamcode.subsystems.ExampleArmSubsystem;

public class SetClawToPosition extends CommandBase {

    private final ExampleArmSubsystem m_exampleArmSubsystem;
    private int position = 0;

    /**
     * Sets the claw to specified angle in degrees (0 to 180).
     *
     * @param exampleArmSubsystem Parent subsystem - ExampleArmSubsystem by default
     * @param position            Degree that is passed into function
     *
     * @see ExampleArmSubsystem#setClawToPosition(double) 
     */
    public SetClawToPosition(ExampleArmSubsystem exampleArmSubsystem, int position){
        this.m_exampleArmSubsystem = exampleArmSubsystem;
        this.position = position;
        addRequirements();
    }

    @Override
    public void initialize(){
        m_exampleArmSubsystem.setClawToPosition(position);
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
