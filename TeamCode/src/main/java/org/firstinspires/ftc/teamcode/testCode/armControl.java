package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tfrec.Detector;

public class armControl extends encoderTest {
    private DcMotor arm;
    private Servo intake;
    private boolean open;

    private Detector tfDetector = null;
    //Create elapsed time variable and an instance of elapsed time
    private ElapsedTime runtime = new ElapsedTime();

    //Convert from the counts per revolution of the encoder to counts per inch
    static final double HD_COUNTS_PER_REV = 537.7; //537.7,,28
    static final double DRIVE_GEAR_REDUCTION = 1; //20.15293;
    static final double WHEEL_CIRCUMFERENCE_MM = 35 * Math.PI;//109.9
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM; //112/109.9
    static final double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;//1.0191*25.4

    public armControl(double power, double inches) {
        //super();

        int target;


        if (opModeIsActive()) {
            // Create target positions
            target = arm.getCurrentPosition() + (int) (inches * DRIVE_COUNTS_PER_IN);

            //arm.setDirection(DcMotorSimple.Direction.REVERSE);
            // set target position
            arm.setTargetPosition(target);

            //switch to run to position mode
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            //run to position at the desiginated power
            arm.setPower(power);


            // wait until both motors are no longer busy running to position
            while (opModeIsActive() && (arm.isBusy())) {
            }

            // set motor power back to 0
            arm.setPower(0);

        }
    }
}
