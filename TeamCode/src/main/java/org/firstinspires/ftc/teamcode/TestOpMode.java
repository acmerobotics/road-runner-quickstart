package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.subsystems.TestSubsystem;
import org.firstinspires.ftc.teamcode.commands.*;

public class TestOpMode extends CommandOpMode {
    private TestSubsystem testSubsystem;

    @Override
    public void initialize() {
        // This will make the default for the test subsystem to stop the motor.
        testSubsystem.setDefaultCommand(new TestStopCommand(testSubsystem));
    }
}
