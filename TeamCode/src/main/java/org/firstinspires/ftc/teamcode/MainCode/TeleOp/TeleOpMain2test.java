package org.firstinspires.ftc.teamcode.MainCode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


/*Buttons
Left and right sticks to drive (Robot is field centric)
options button to reset gyro
left trigger to run intake grabber, right trigger to run backwards
a to initiate transfer
b to reset transfer
 */
@TeleOp(name="MainTeleOpTest", group="Linear Opmode")
public class TeleOpMain2test extends LinearOpMode
{
    //Variables ____________________________________________________________________________________
    DcMotorEx intake_elbow, outtake_elbow, hang_arm;
    DcMotor front_left, back_left, front_right, back_right, intake_grabber;
    Servo left_intake, right_intake, outtake_wrist, drone_launcher;
    double intakeServoStart;
    double outtakeServoDrop = .4;
    double intakeServoTransfer = .2;
    double outtakeServoTransfer = .2;
    int intakeMotorTransfer = -125;
    int outtakeMotorTransfer = 200;
    int intakeMotorStart;
    double power = .5;
    IMU imu;
    //private Gamepad lastGamepad;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException //START HERE
    {
        HardwareSetupMotors();
        HardwareSetupServos();
        ImuSetup();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive())
        {
            //intake_grabber.setPower(gamepad1.left_trigger);
            //intake_grabber.setPower(-gamepad1.right_trigger);


            int pos = intake_elbow.getCurrentPosition();
            telemetry.addData("pos", pos);
            telemetry.update();
            RunIntakeGrabber();
            if (gamepad1.a/* && !lastGamepad.a*/)
            {
                //lastGamepad.copy(gamepad1);
                InitiateTransfer();
            }
            if (gamepad1.b/* && !lastGamepad.b*/)
            {
                //lastGamepad.copy(gamepad1);
                ResetTransfer();
            }


            //lastGamepad.copy(gamepad1);
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

        intakeMotorStart = intake_elbow.getCurrentPosition();
    }
    private void HardwareSetupServos()
    {
        left_intake = hardwareMap.get(Servo.class,"left_intake");
        right_intake = hardwareMap.get(Servo.class,"right_intake");
        outtake_wrist = hardwareMap.get(Servo.class,"outtake_wrist");
        drone_launcher = hardwareMap.get(Servo.class,"drone_launcher");

        intakeServoStart = left_intake.getPosition();
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
    private void RunIntakeGrabber()
    {
        intake_grabber.setPower(gamepad1.left_trigger);
        intake_grabber.setPower(-gamepad1.right_trigger);
    }
    private void InitiateTransfer()
    {
        //outtake_elbow.setTargetPosition(outtakeMotorTransfer);
        //outtake_elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //outtake_elbow.setPower(power);
        /*outtake_wrist.setPosition(outtakeServoTransfer);

        intake_elbow.setTargetPosition(intakeMotorTransfer);
        intake_elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake_elbow.setPower(power);
        left_intake.setPosition(intakeServoTransfer);*/
        right_intake.setPosition(0);
    }
    private void ResetTransfer()
    {
        /*left_intake.setPosition(intakeServoStart);
        right_intake.setPosition(-intakeServoStart);
        intake_elbow.setTargetPosition(intakeMotorStart);
        intake_elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake_elbow.setPower(power);

        outtake_wrist.setPosition(outtakeServoDrop);*/
        right_intake.setPosition(1);
    }
}
