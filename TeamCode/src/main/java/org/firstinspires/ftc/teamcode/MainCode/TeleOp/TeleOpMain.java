package org.firstinspires.ftc.teamcode.MainCode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="DriveTrain", group="Linear Opmode")
public class TeleOpMain extends LinearOpMode
{
    //Variables ____________________________________________________________________________________
    DcMotorEx intake_elbow, intake_grabber, outtake_elbow;
    DcMotor front_left, back_left, front_right, back_right;
    Servo left_intake, right_intake, outtake_wrist;
    double intakeServoStart;
    double outtakeServoStart;
    private ElapsedTime runtime = new ElapsedTime();


    public void runOpMode()
    {
        HardwareSetupMotors();
        HardwareSetupServos();

        waitForStart();
    }

    // Methods______________________________________________________________________________________

    private void HardwareSetupMotors()
    {
        intake_elbow = hardwareMap.get(DcMotorEx.class, "intake_elbow");
        intake_grabber = hardwareMap.get(DcMotorEx.class, "intake_grabber");
        outtake_elbow = hardwareMap.get(DcMotorEx.class, "outtake_elbow");

        front_left  = hardwareMap.get(DcMotor.class, "leftfront_drive");
        back_left  = hardwareMap.get(DcMotor.class, "leftback_drive");
        front_right = hardwareMap.get(DcMotor.class, "rightfront_drive");
        back_right = hardwareMap.get(DcMotor.class, "rightback_drive");

        intake_elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MotorInit(intake_elbow);
        MotorInit(intake_grabber);
        MotorInit(outtake_elbow);
    }
    private void HardwareSetupServos()
    {
        left_intake = hardwareMap.get(Servo.class,"left_intake");
        right_intake = hardwareMap.get(Servo.class,"right_intake");
        outtake_wrist = hardwareMap.get(Servo.class,"outtake_wrist");
    }
    private void MotorInit(DcMotorEx motor)
    {
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setPower(0.0);
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(0);
        motor.setVelocityPIDFCoefficients(25.0,0.0,0.0,0.0);
        motor.setPositionPIDFCoefficients(25.0);
        motor.setPower(1.0);
    }
    private void ServoStartingPos()
    {
        left_intake.setPosition(intakeServoStart);
        right_intake.setPosition(-intakeServoStart);
        outtake_wrist.setPosition(outtakeServoStart);
    }
}
