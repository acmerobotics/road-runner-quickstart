package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "nowWithArm")
public class nowWithArm extends LinearOpMode {

    private IMU imu;
    private DcMotor backRight;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor arm;

    private void setupMovement() {
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

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        arm = hardwareMap.get(DcMotor.class, "arm");
        setupMovement();

        // Arm SetUp
        final int ARMMIN = arm.getCurrentPosition();
        final int ARMMAX = ARMMIN - 3470;
        final int INCREMENT = 10;
        arm.setPower(0.5);

        waitForStart();
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive()) {
            chassisMovement();
            armMovement(ARMMAX, ARMMIN, INCREMENT);
            printPosition();
        }
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
        while(!arm.isBusy()) {
            if (gamepad1.dpad_down && (ARMMIN <= (armPosition + INCREMENT))) {      // if (DPAD-down) is being pressed and if not yet the min
                arm.setTargetPosition(armPosition + INCREMENT);                     // Position in
            } else if (gamepad1.dpad_up && (ARMMAX >= (armPosition - INCREMENT))) { // if (DPAD-up) is being pressed and if not yet max
                arm.setTargetPosition(armPosition - INCREMENT);                     // Position Out
            }
        }
    }
    private void armHotKeys(int[] targets) {
        
        arm.setTargetPosition();
    }
    private void printPosition() {
        int position = arm.getCurrentPosition();
        telemetry.addData("Encoder position, ", position);
        telemetry.update();
    }

}