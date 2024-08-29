package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.TestSubsystem;
import org.firstinspires.ftc.teamcode.commands.*;

public class TestOpMode extends CommandOpMode {
    private TestSubsystem m_testSubsystem;

    @Override
    public void initialize() {
        // This is for the subsystem requirements
        m_testSubsystem = new TestSubsystem(hardwareMap.get(DcMotor.class, "testMotor"));

        // This will make the default for the test subsystem to stop the motor.
        m_testSubsystem.setDefaultCommand(new TestForwardCommand(m_testSubsystem));
    }
}
