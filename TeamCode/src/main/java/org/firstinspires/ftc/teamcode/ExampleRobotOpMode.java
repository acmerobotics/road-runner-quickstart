package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.ExampleArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TestSubsystem;
import org.firstinspires.ftc.teamcode.commands.*;

@TeleOp(name= "ExampleRobotOpMode", group="Examples")
public class ExampleRobotOpMode extends CommandOpMode {
    private ExampleArmSubsystem m_exampleArmSubsystem;
    private Gamepad gamepad;

    @Override
    public void initialize() {
        m_exampleArmSubsystem = new ExampleArmSubsystem(
                hardwareMap.get(DcMotor.class, "LeftArmMotor"),
                hardwareMap.get(DcMotor.class, "RightArmMotor"),
                hardwareMap.get(Servo.class, "ArmServo"));

        // This will make the default for the test subsystems.
        m_exampleArmSubsystem.setDefaultCommand(new SetToOriginalPosition(m_exampleArmSubsystem));
    }
}
