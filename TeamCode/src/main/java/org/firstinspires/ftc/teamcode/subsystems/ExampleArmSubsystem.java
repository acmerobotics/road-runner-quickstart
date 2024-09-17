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

    /**
     * [BETA]
     * <p>
     * Schematic of an arm subsystem to be modified as needed.
     * Controls a dual-motor arm with many methods to manipulate it with PID.
     * As of version BETA, the claw has yet to be implemented.
     *
     * @param leftMotor  Left motor on the dual-motor arm
     * @param rightMotor Right motor on the dual-motor arm
     * @param clawServo  Servo to control the claw
     *
     * @see ExampleArmSubsystem#moveArmToPosition(int)
     * @see ExampleArmSubsystem#liftArmToMax()
     * @see ExampleArmSubsystem#resetToOriginalPosition()
     */
    public ExampleArmSubsystem(DcMotor leftMotor, DcMotor rightMotor, Servo clawServo) {
        // Initializing objects
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;

        this.clawServo = clawServo;

        // Setting motor direction
        this.leftMotor.setDirection(DcMotor.Direction.REVERSE);

        // Setting RunMode - This is to set position for PID <- todo LOOK FOR OTHER DEPENDENCIES
        this.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Lifts up the dual-motor arm to the max position specified in <code>Constants.java</code>.
     * As of initial implementation, the code may not work as intended.
     *
     * @see org.firstinspires.ftc.teamcode.Constants
     * @see DcMotor#setTargetPosition(int)
     */
    public void liftArmToMax() {
        leftMotor.setTargetPosition(MAX_ARM_POSITION);
        rightMotor.setTargetPosition(MAX_ARM_POSITION);
    }

    /**
     * Sets the dual-motor arm to the specified position
     *
     * @param position  The position parameter in <code>setTargetPosition()</code>
     *
     * @see DcMotor#setTargetPosition(int)
     */
    public void moveArmToPosition(int position) {
        leftMotor.setTargetPosition(position);
        rightMotor.setTargetPosition(position);
    }

    /**
     * Sets the dual-motor arm to the original position specified in <code>Constants.java</code>.
     * As of initial implementation, the code may not work as intended.
     *
     * @see org.firstinspires.ftc.teamcode.Constants
     * @see DcMotor#setTargetPosition(int)
     */
    public void resetToOriginalPosition() {
        leftMotor.setTargetPosition(ORIGINAL_ARM_POSITION);
        rightMotor.setTargetPosition(ORIGINAL_ARM_POSITION);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}