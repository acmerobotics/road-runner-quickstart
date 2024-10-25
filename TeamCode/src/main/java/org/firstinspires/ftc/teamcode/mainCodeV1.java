package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.ColorSensor;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;




@TeleOp(name = "mainCodeV1")
public class mainCodeV1 extends LinearOpMode {
    // variable declaration
    private IMU imu;
    private DcMotor backRight;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor arm;
    private ColorSensor colorDetector;
    int ARMMIN;
    int ARMMAX;
    int INCREMENT;


    private void setupChassis() {
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void chassisMovement() {
        float y = gamepad1.left_stick_y;
        float x = -gamepad1.left_stick_x;
        float t = -gamepad1.right_stick_x;
        double botHeading;
        double rotX;
        double rotY;
        double denominator;
        double frontLeftPower;
        double backLeftPower;
        double frontRightPower;
        double backRightPower;
        if (gamepad1.start) {
            imu.resetYaw();
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
    }

    private void armMovement(int ARMMAX, int ARMMIN, int INCREMENT) {
        int armPosition = arm.getCurrentPosition();
        if (gamepad1.dpad_down) {       // if (DPAD-down) is being pressed and if not yet the min
            armPosition += INCREMENT;   // Position in
        } else if (gamepad1.dpad_up) {  // if (DPAD-up) is being pressed and if not yet max
            armPosition -= INCREMENT;   // Position Out
        }
        armPosition = Math.max(Math.min(armPosition, ARMMIN), ARMMAX);
        arm.setTargetPosition(armPosition);
    }

    private void printPosition() {
        int position = arm.getCurrentPosition();
        int target = arm.getTargetPosition();

        telemetry.addData("Encoder position, ", position);
        telemetry.addData("Encoder target, ", target);

        telemetry.addData("Red", colorDetector.red());
        telemetry.addData("Green", colorDetector.green());
        telemetry.addData("Blue", colorDetector.blue());
        telemetry.update();
    }

    private void hardwareMapping() {
        imu = hardwareMap.get(IMU.class, "imu");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        arm = hardwareMap.get(DcMotor.class, "arm");
        colorDetector = hardwareMap.get(ColorSensor.class, "colorDetector");
    }

    private void initializeSetUp() {
        hardwareMapping();
        setupChassis();
        armSetup();
    }

    private void armSetup() {
        arm.setPower(1);
        ARMMIN = arm.getCurrentPosition() - 3;
        ARMMAX = ARMMIN - 3200;
        INCREMENT = 250;
    }

    private void startSetUp() {
        arm.setTargetPosition(arm.getCurrentPosition());
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    @Override
    public void runOpMode() {
        initializeSetUp();
        waitForStart();
        startSetUp();
        while (opModeIsActive()) {
            chassisMovement();
            armMovement(ARMMAX, ARMMIN, INCREMENT);
            printPosition();
        }
    }
}