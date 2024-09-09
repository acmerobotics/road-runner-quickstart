package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.ExampleArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TestSubsystem;
import org.firstinspires.ftc.teamcode.commands.*;

public class TestOpMode extends CommandOpMode {
    private TestSubsystem m_testSubsystem;
    private ExampleArmSubsystem m_exampleArmSubsystem;

    @Override
    public void initialize() {
        // This is for the subsystem requirements
        m_testSubsystem = new TestSubsystem(hardwareMap.get(DcMotor.class, "testMotor"));
        m_exampleArmSubsystem = new ExampleArmSubsystem(
                hardwareMap.get(DcMotor.class, "LeftArmMotor"),
                hardwareMap.get(DcMotor.class, "RightArmMotor"),
                hardwareMap.get(Servo.class, "ArmServo"));

        // This will make the default for the test subsystems.
        m_testSubsystem.setDefaultCommand(new TestForwardCommand(m_testSubsystem));
        m_exampleArmSubsystem.setDefaultCommand(new SetToOriginalPosition(m_exampleArmSubsystem));
    }
}
