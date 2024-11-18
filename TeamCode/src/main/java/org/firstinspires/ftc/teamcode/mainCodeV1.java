package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


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

    private void hardwareMapping() {
        imu = hardwareMap.get(IMU.class, "imu");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        arm = hardwareMap.get(DcMotor.class, "arm");
        colorDetector = hardwareMap.get(ColorSensor.class, "colorDetector");
    }

    private void armSetup() {
        arm.setPower(1);
        ARMMIN = arm.getCurrentPosition() - 3;
        ARMMAX = ARMMIN - 3000;
        INCREMENT = 250;
    }

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

    private void initializeAndSetUp() {
        hardwareMapping();
        setupChassis();
        armSetup();
    }

    private void chassisMovement(float y, float x, float t) {
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

    private void armMovement() {
        int armPosition = arm.getCurrentPosition();
        if (gamepad1.dpad_down) {       // if (DPAD-down) is being pressed and if not yet the min
            armPosition += INCREMENT;   // Position in
        } else if (gamepad1.dpad_up) {  // if (DPAD-up) is being pressed and if not yet max
            armPosition -= INCREMENT;   // Position Out
        }
        armPosition = Math.max(Math.min(armPosition, ARMMIN), ARMMAX);  //clamp the values to be between min and max
        arm.setTargetPosition(armPosition);
    }

    private void postStartSetUp() {
        arm.setTargetPosition(arm.getCurrentPosition());
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void printThings() {
        telemetry.addData("Color: ", colorDetection());
        telemetry.addData("difference", distance((float) imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), 90f));
        telemetry.addData("Heading: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
    }
    private static float distance(float alpha, float beta) {
        float phi = (beta - alpha) % 360; // Raw difference in range [-359, 359]
        float distance = phi > 180 ? phi - 360 : (phi < -180 ? phi + 360 : phi);
        return distance;
    }

    private String colorDetection() {
        String[] colors = {"Yellow", "Blue", "Red"};
        String color = "";
        float ratioGreenOverRed = ((float)colorDetector.green() / colorDetector.red());
        float ratioBlueOverRed = ((float)colorDetector.blue() / colorDetector.red());

        if ((ratioGreenOverRed >= 1.1 && ratioGreenOverRed <= 2.0) &&
                (ratioBlueOverRed >= 0.1 && ratioBlueOverRed <= 0.8)) {
            color = colors[0]; // Yellow
        }
        if ((ratioGreenOverRed >= 1.5 && ratioGreenOverRed <= 2.7) &&
                (ratioBlueOverRed >= 2.0 && ratioBlueOverRed <= 10.0)) {
            color = colors[1]; // Blue
        }
        if ((ratioGreenOverRed >= 0.2 && ratioGreenOverRed <= 1) &&
                (ratioBlueOverRed >= 0.1 && ratioBlueOverRed <= 0.8)) {
            color = colors[2]; // Red
        }
        return color;
    }

    private void rotateTo(double targetDegree) {
        double botHeading;
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        float direction = distance((float)botHeading, (float)targetDegree);
        float power = Math.max(Math.min(0.5f, (float) (0.001 * Math.pow(direction, 2))), 0.225f); // 1 is clockwise, -1 is counterclock minimum is 0.1 (might need to be lower) and max is 0.5
        if (Math.abs(direction) < 1f) {     // if the angle is less than 1 then poweroff
            power = 0f;
        }
        power = direction < 0 ? power * -1: power;
        chassisMovement(0,0, power);
    }
    @Override
    public void runOpMode() {
        initializeAndSetUp();
        waitForStart();
        postStartSetUp();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                rotateTo(90);
            } else {
                chassisMovement(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            }
            armMovement();
            printThings();
        }
    }
}