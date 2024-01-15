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
    public static double outtakeServoDrop = .13;
    public static double intakeServoTransfer = .95;
    public static double outtakeServoTransfer = .53;
    public static int intakeMotorTransfer = -111;
    public static int outtakeMotorTransfer = 0;
    public static int intakeMotorStart;

    public static double power = .5;
    public static boolean intakeToggleL = false;
    public static boolean intakeToggleR = false;
    public static boolean manualToggle = false;
    public static boolean allowPIDF = true;
    public static int lowDropPos = 800;
    public static int midDropPos = 1600;
    public static int highDropPos = 2400;
    public static double offset = 0.05;
    public static double slidePower = 1;
    public static double pVal = 1.26;
    public static double iVal = 0.126;
    public static double dVal = 0;
    public static double fVal = 12.6;
    public static double posVal = 5;
    public static double motorVel = 2600;
    public static double speedVar = .6;
    public static int resetVar1 = -90;
    public static double rpower1 = .7;
    public static int resetVar2 = -80;
    public static double rpower2 = .6;
    int intakePos;
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

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            MoveRobot();

            if (currentGamepad1.a && !previousGamepad1.a) {
                InitiateTransfer();
            } else if (currentGamepad1.b && !previousGamepad1.b) {
                ResetTransfer();
            }
            if (gamepad1.dpad_left) {
                intakePos = intake_elbow.getCurrentPosition();
                intake_elbow.setTargetPosition(intakePos - 1);
                intake_elbow.setPower(.5);
                intake_elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad1.dpad_up) {
                intakePos = intake_elbow.getCurrentPosition();
                intake_elbow.setTargetPosition(intakePos + 25);
                intake_elbow.setPower(1);
                intake_elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
                outtake_wrist.setPosition(outtakeServoTransfer);
            }
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                NoDrop();
            }
            ManualSlidePos();
            intake_grabber.setPower(gamepad2.right_stick_y);

            int currentPos = intake_elbow.getCurrentPosition();
            telemetry.addData("pos", currentPos);
            telemetry.update();
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

        if (allowPIDF)
        {
            outtake_elbow.setVelocityPIDFCoefficients(pVal,iVal,dVal,fVal);
        }
        outtake_elbow.setPositionPIDFCoefficients(posVal);
        outtake_elbow.setVelocity(motorVel);

        intake_elbow.setVelocityPIDFCoefficients(25,0,0,0);
        intake_elbow.setPositionPIDFCoefficients(25);

        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
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

        if (gamepad1.y) {
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

        front_left.setPower(frontLeftPower*speedVar);
        back_left.setPower(backLeftPower*speedVar);
        front_right.setPower(frontRightPower*speedVar);
        back_right.setPower(backRightPower*speedVar);
    }
    private void InitiateTransfer()
    {
        //outtake_elbow.setTargetPosition(outtakeMotorTransfer);
        //outtake_elbow.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //outtake_elbow.setPower(slidePower);
        outtake_wrist.setPosition(outtakeServoTransfer);

        intake_elbow.setTargetPosition(intakeMotorTransfer);
        intake_elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake_elbow.setPower(power);
        left_intake.setPosition(intakeServoTransfer);
        right_intake.setPosition(intakeServoTransfer);
    }
    private void ResetTransfer()
    {
        /*
        while (outtake_wrist.getPosition() > (outtakeServoDrop + offset))
        {
            outtake_wrist.setPosition(outtakeServoDrop);
        }
         */
        outtake_wrist.setPosition(outtakeServoDrop);
        intake_elbow.setTargetPosition(resetVar1);
        intake_elbow.setPower(rpower1);
        intake_elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        right_intake.setPosition(intakeServoStart);

        intake_elbow.setTargetPosition(resetVar2);
        intake_elbow.setPower(rpower2);
        intake_elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (intake_elbow.isBusy())
        {

        }

        sleep(400);

        intake_elbow.setTargetPosition(-40);
        intake_elbow.setPower(.3);
        intake_elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (intake_elbow.isBusy())
        {

        }
        sleep(400);

        intake_elbow.setTargetPosition(intakeMotorStart);
        intake_elbow.setPower(.3);
        intake_elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*intake_elbow.setTargetPosition(-35);
        intake_elbow.setPower(0.2);
        intake_elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intake_elbow.setTargetPosition(intakeMotorStart);
        intake_elbow.setPower(0.2);
        intake_elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
    }
    private void AdjustDown()
    {
        intake_elbow.setTargetPosition(intake_elbow.getCurrentPosition()-1);
        intake_elbow.setPower(.5);
        intake_elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void AdjustUp()
    {
        intake_elbow.setTargetPosition(intake_elbow.getCurrentPosition()+1);
        intake_elbow.setPower(.5);
        intake_elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void NoDrop()
    {
        outtake_wrist.setPosition(outtakeServoDrop);
    }
    private void ManualSlidePos()
    {
        outtake_elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake_elbow.setPower(-gamepad1.right_stick_y);
    }
}
