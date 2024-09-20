package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class FOTeleOp extends OpMode {

    IMU imu;
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    DcMotorEx leftSlidesOuttakeMotor;
    DcMotorEx rightSlidesOuttakeMotor;
    DcMotorEx intakeSlidesMotor;
    DcMotorEx turretMotor;
    double y = 0;
    double x = 0;
    double rx = 0;
    CRServo servoIntakeSpin;
    Servo servoIntakeRotate;
    Servo servoOuttakeClamp;
    Servo servoOuttakeRotate;
    Servo servoOuttakeArmRotate;


    //Initialize all servos
    //Initialize all sensors
    //Initialize all variables

    //This is an example state machine for the arm
    public enum TransferState {
        IntakeSlidesRetract,
        IntakeRotate,
        OuttakeHold,
        OuttakeRotate
    };
    public enum SampleDrop {
        IntakeSlidesRetract,
        IntakeRotate,
        OuttakeHold,
        OuttakeRotate
    };

    //This is the timer for the arm
    ElapsedTime armTimer = new ElapsedTime();
    //This is the starting state
    TransferState transferState = TransferState.IntakeSlidesRetract;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        leftSlidesOuttakeMotor = (DcMotorEx) hardwareMap.dcMotor.get("outtakeLeft");
        rightSlidesOuttakeMotor = (DcMotorEx) hardwareMap.dcMotor.get("outtakeRight");
        intakeSlidesMotor = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        turretMotor = (DcMotorEx) hardwareMap.dcMotor.get("turret");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlidesOuttakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlidesOuttakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSlidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoIntakeSpin = (CRServo) hardwareMap.servo.get("intakeSpin");
        servoIntakeRotate = hardwareMap.servo.get("intakeRotate");
        servoOuttakeClamp = hardwareMap.servo.get("outtakeClamp");
        servoOuttakeRotate = hardwareMap.servo.get("outtakeRotate");
        servoOuttakeArmRotate = hardwareMap.servo.get("outtakeArmRotate");


        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
    }

    @Override
    public void start() {
        servoIntakeRotate.setPosition(0.4); //0.4 to 0.8
        servoOuttakeClamp.setPosition(0.4); //0.4 to 0.5
        servoOuttakeRotate.setPosition(0.4); //0.4 to 0.6
        servoOuttakeArmRotate.setPosition(0.4); //0.4 to 0.8
    }

    @Override
    public void loop() {
        switch (transferState) {
            case IntakeSlidesRetract:
               // if intakeColorSensor.distance() <= 1{


            //}
            case IntakeRotate:
                
            case OuttakeHold:
            case OuttakeRotate:
        }



        if (gamepad1.right_trigger > 0) {
            y = -gamepad1.left_stick_y - gamepad1.right_stick_y; // Remember, this is reversed!
            x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            rx = -gamepad1.right_stick_x;
        } else if (gamepad1.left_trigger > 0) {
            y = -0.25 * (gamepad1.left_stick_y + gamepad1.right_stick_y); // Remember, this is reversed!
            x = 0.25 * gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            rx = -0.35 * gamepad1.right_stick_x;
        } else {
            y = -0.5 * (gamepad1.left_stick_y + gamepad1.right_stick_y); // Remember, this is reversed!
            x = 0.5 * gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            rx = -0.65 * gamepad1.right_stick_x;
        }

        if (gamepad1.back) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.toRadians(90);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }
}