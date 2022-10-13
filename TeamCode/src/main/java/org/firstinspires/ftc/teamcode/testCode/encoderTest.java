package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tfrec.Detector;

import java.util.List;
import java.util.concurrent.TimeUnit;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name = "Auto Test")

public class encoderTest extends LinearOpMode {


    private DcMotor arm;
    //private DcMotor Arm;
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

//35mm = 1.378 inches

    // Drive function with 3 parameters
    private void armControl(double power, double inches) {
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



    @Override
    public void runOpMode() throws InterruptedException {
        //telemetry.addData("Status" , "Initialized");
        //telemetry.update();

        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(Servo.class, "intake");


        intake.setDirection(Servo.Direction.REVERSE);
        intake.setPosition(0.0);


//maybe reverse?


        waitForStart();
        runtime.reset();




        if (intake.getPosition() == 0.0) {
            open = true;

        }

        if (open) {
            intake.setPosition(0.5);
            sleep(1000);
            open = false;
        }

        armControl(0.5, 10);



        if (arm.getCurrentPosition() == arm.getTargetPosition()) {

                open = true;
                intake.setPosition(0.0);
                sleep(1000);

            }
        }






    }




