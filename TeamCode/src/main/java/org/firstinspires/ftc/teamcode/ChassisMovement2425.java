package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import kotlin.math.UMathKt;

@TeleOp(name = "ChassisMovement2425")
public class ChassisMovement2425 extends LinearOpMode {
    private IMU imu;
    private DcMotor backRight;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private Servo clawUpDown;

    private void setupMovement() {
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");

        clawUpDown = hardwareMap.get(Servo.class, "servoUpDown");

        setupMovement();
        waitForStart();
        if (isStopRequested()) {
            // return???
        }
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                manualMove();
            }
        }
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }


    private void manualMove() {
        float y;
        float x;
        float t;
        double botHeading;
        double rotX;
        double rotY;
        double denominator;
        double frontLeftPower;
        double backLeftPower;
        double frontRightPower;
        double backRightPower;

        double clawYPos = 0.3;
        double clawMax = 0.8; //adjust this
        double clawMin = 0.2; //adjust this

        y = gamepad1.left_stick_y;
        x = -gamepad1.left_stick_x;
        t = -gamepad1.right_stick_x;
        if (gamepad1.start) {
            imu.resetYaw();
        }

        if (gamepad1.dpad_left && !gamepad1.dpad_right) {
            //claw down

            clawYPos = clamp(clawYPos - 0.01, clawMin, clawMax);

        }else if (gamepad1.dpad_right && !gamepad1.dpad_left){
            //claw up
            clawYPos = clamp(clawYPos + 0.01, clawMin, clawMax);
        }

        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        rotX = x * Math.cos(-botHeading / 180 * Math.PI) - y * Math.sin(-botHeading / 180 * Math.PI);
        rotY = x * Math.sin(-botHeading / 180 * Math.PI) + y * Math.cos(-botHeading / 180 * Math.PI);
        rotX = rotX * 1.1;
        denominator = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(rotY) + Math.abs(rotX) + Math.abs(t), 1));
        frontLeftPower = (rotY + rotX + t) / denominator;
        backLeftPower = (rotY - (rotX - t)) / denominator;
        frontRightPower = (rotY - (rotX + t)) / denominator;
        backRightPower = (rotY + (rotX - t)) / denominator;
        frontLeft.setPower(0.75 * frontLeftPower);
        backLeft.setPower(0.75 * backLeftPower);
        frontRight.setPower(0.75 * frontRightPower);
        backRight.setPower(0.75 * backRightPower);
        clawUpDown.setPosition(clawYPos);
    }
}