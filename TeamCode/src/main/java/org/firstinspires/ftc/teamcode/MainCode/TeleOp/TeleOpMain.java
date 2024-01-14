package org.firstinspires.ftc.teamcode.MainCode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.acmerobotics.dashboard.FtcDashboard;

/*Buttons
Left and right sticks to drive (Robot is field centric)
options button to reset gyro
left bumper tp toggle forward intake grabber, right bumper to toggle backwards
a to initiate transfer
b to reset transfer
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="MainTeleOp", group="Linear Opmode")
@Config
public class TeleOpMain extends LinearOpMode
{
    //Variables ____________________________________________________________________________________
    DcMotorEx intake_elbow, outtake_elbow, hang_arm;
    DcMotor front_left, back_left, front_right, back_right, intake_grabber;
    Servo left_intake, right_intake, outtake_wrist, drone_launcher;
    public static double intakeServoStart = .857;
    public static double outtakeServoDrop = .4;
    public static double intakeServoTransfer = .95;
    public static double outtakeServoTransfer = .245;
    public static int intakeMotorTransfer = -112;
    public static int outtakeMotorTransfer = 0;
    public static int intakeMotorStart;

    public static double power = .5;
    public static boolean intakeToggleL = false;
    public static boolean intakeToggleR = false;
    public static int lowDropPos = 800;
    public static int midDropPos = 1600;
    public static int highDropPos = 2400;
    IMU imu;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException //START HERE
    {
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        HardwareSetupMotors();
        HardwareSetupServos();
        ImuSetup();
        right_intake.setPosition(intakeServoStart);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive())
        {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            MoveRobot();

            if (currentGamepad1.a && !previousGamepad1.a)
            {
                InitiateTransfer();
            }
            else if (currentGamepad1.b && !previousGamepad1.b)
            {
                ResetTransfer();
            }

            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper)
            {
                intakeToggleL = !intakeToggleL;
            }
            if (intakeToggleL)
            {
                intake_grabber.setPower(.75);
            }
            else
            {
                intake_grabber.setPower(0);
            }

            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper)
            {
                intakeToggleR = !intakeToggleR;
            }
            if (intakeToggleR)
            {
                intake_grabber.setPower(-.75);
            }
            else
            {
                intake_grabber.setPower(0);
            }
        }
    }

    // Methods______________________________________________________________________________________

    public void HardwareSetupMotors()
    {
        intake_elbow = hardwareMap.get(DcMotorEx.class, "intake_elbow");
        outtake_elbow = hardwareMap.get(DcMotorEx.class, "outtake_elbow");
        hang_arm = hardwareMap.get(DcMotorEx.class, "hang_arm");

        front_left  = hardwareMap.get(DcMotor.class, "front_left");
        back_left  = hardwareMap.get(DcMotor.class, "back_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_right = hardwareMap.get(DcMotor.class, "back_right");

        intake_grabber = hardwareMap.get(DcMotor.class, "intake_grabber");

        MotorInit(intake_elbow);
        MotorInit(hang_arm);
        MotorInit(outtake_elbow);

        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);
        outtake_elbow.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    private void HardwareSetupServos()
    {
        left_intake = hardwareMap.get(Servo.class,"left_intake");
        right_intake = hardwareMap.get(Servo.class,"right_intake");
        outtake_wrist = hardwareMap.get(Servo.class,"outtake_wrist");
        drone_launcher = hardwareMap.get(Servo.class,"drone_launcher");
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
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private void ImuSetup()
    {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }
    private void MoveRobot()
    {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        front_left.setPower(frontLeftPower);
        back_left.setPower(backLeftPower);
        front_right.setPower(frontRightPower);
        back_right.setPower(backRightPower);
    }
    private void InitiateTransfer()
    {
        outtake_elbow.setTargetPosition(outtakeMotorTransfer);
        outtake_elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtake_elbow.setPower(power);
        outtake_wrist.setPosition(outtakeServoTransfer);

        intake_elbow.setTargetPosition(intakeMotorTransfer);
        intake_elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake_elbow.setPower(power);
        left_intake.setPosition(intakeServoTransfer);
        right_intake.setPosition(intakeServoTransfer);
    }
    private void ResetTransfer()
    {
        intake_elbow.setTargetPosition((intake_elbow.getCurrentPosition() + 10));
        intake_elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake_elbow.setPower(0.1);

        right_intake.setPosition(intakeServoStart);

        intake_elbow.setTargetPosition(intakeMotorStart);
        intake_elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake_elbow.setPower(0.1);

        outtake_wrist.setPosition(outtakeServoDrop);
    }
    private void LowDrop()
    {
        outtake_elbow.setTargetPosition(lowDropPos);
        outtake_elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtake_elbow.setPower(power);
    }
    private void MidDrop()
    {
        outtake_elbow.setTargetPosition(midDropPos);
        outtake_elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtake_elbow.setPower(power);
    }
    private void HighDrop()
    {
        outtake_elbow.setTargetPosition(highDropPos);
        outtake_elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtake_elbow.setPower(power);
    }
}
