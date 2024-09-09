package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class ExampleArmSubsystem extends SubsystemBase {

    // Creating the motor objects
    private final DcMotor leftMotor;
    private final DcMotor rightMotor;

    //Creating the servo objects
    private final Servo clawServo;

    public ExampleArmSubsystem(DcMotor leftMotor, DcMotor rightMotor, Servo clawServo) {
        // Initializing objects
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;

        this.clawServo = clawServo;

        // Setting motor direction
        this.leftMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void liftArmToMax() {
        /**
         * @see MAX_ARM_POSITION found in {@link org.firstinspires.ftc.teamcode.Constants Constants.java}
         */
        leftMotor.setTargetPosition(MAX_ARM_POSITION);
        rightMotor.setTargetPosition(MAX_ARM_POSITION);
    }

    public void moveArmToPosition(int position) {
        leftMotor.setTargetPosition(position);
    }

    public void ResetToOriginalPosition() {
        leftMotor.setTargetPosition(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}