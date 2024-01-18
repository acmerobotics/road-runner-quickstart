package org.firstinspires.ftc.teamcode.MainCode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
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
public class TeleOpMain extends LinearOpMode {
    //Variables ____________________________________________________________________________________
    DcMotorEx intake_elbow, outtake_elbow, hang_arm;
    DcMotor front_left, back_left, front_right, back_right, intake_grabber;
    Servo left_intake, right_intake, outtake_wrist, drone_launcher;
    public static double intakeServoStart = .857;
    public static double outtakeServoDrop = .13;
    public static double intakeServoTransfer = .95;
    public static double outtakeServoTransfer = .53;
    public static int intakeMotorTransfer = -116;
    public static int outtakeMotorTransfer = 0;
    public static int intakeMotorStart;
    public static double speedVar = 1;
    public static int resetVar1 = -90;
    public static int resetVar2 = -40;
    int intakePos;
    IMU imu;
    private ElapsedTime runtime = new ElapsedTime();


    private PIDController controller;
    public static double p = 0.02, i = 0, d = 0.0002;
    public static double f = -0.15;
    private final double ticks_in_degree = 144.0 / 180.0;
    public static int target;
    public static double offset = -25;
    int armPos;
    double pid, targetArmAngle, ff, currentArmAngle, intakeArmPower;
    @Override
    public void runOpMode() throws InterruptedException //START HERE
    {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

            if (currentGamepad1.a && !previousGamepad1.a) {
                InitiateTransfer();
            }
            if (currentGamepad1.b && !previousGamepad1.b) {
                ResetTransfer1();
            }
            if (currentGamepad1.x && !previousGamepad1.x) {
                ResetTransfer2();
            }
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                AdjustDown();
            }
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                AdjustUp();
            }
            if (gamepad1.dpad_right) {
                outtake_wrist.setPosition(outtakeServoTransfer);
                telemetry.addData("dpad right", "pressed");
            }
            if (gamepad1.dpad_left) {
                NoDrop();
                telemetry.addData("dpad left", "pressed");
            }

            MoveRobot();

            ManualSlidePos();

            //PID stuff
            controller.setPID(p, i, d);
            armPos = intake_elbow.getCurrentPosition();
            pid = controller.calculate(armPos, target);
            targetArmAngle = Math.toRadians((target - offset) / ticks_in_degree);
            ff = Math.cos(targetArmAngle) * f;
            currentArmAngle = Math.toRadians((armPos - offset) / ticks_in_degree);

            intakeArmPower = pid + ff;

            intake_elbow.setPower(intakeArmPower);
            hang_arm.setPower(intakeArmPower);

            TelemetryData();
        }
    }
    // Methods______________________________________________________________________________________

    public void HardwareSetupMotors() {
        intake_elbow = hardwareMap.get(DcMotorEx.class, "intake_elbow");
        outtake_elbow = hardwareMap.get(DcMotorEx.class, "outtake_elbow");
        hang_arm = hardwareMap.get(DcMotorEx.class, "hang_arm");

        front_left = hardwareMap.get(DcMotor.class, "leftfront_drive");
        back_left = hardwareMap.get(DcMotor.class, "leftback_drive");
        front_right = hardwareMap.get(DcMotor.class, "rightfront_drive");
        back_right = hardwareMap.get(DcMotor.class, "rightback_drive");

        intake_grabber = hardwareMap.get(DcMotor.class, "intake_grabber");

        MotorInit(intake_elbow);
        MotorInit(hang_arm);
        MotorInit(outtake_elbow);

        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        outtake_elbow.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void HardwareSetupServos() {
        left_intake = hardwareMap.get(Servo.class, "left_intake");
        right_intake = hardwareMap.get(Servo.class, "right_intake");
        outtake_wrist = hardwareMap.get(Servo.class, "outtake_wrist");
        drone_launcher = hardwareMap.get(Servo.class, "drone_launcher");
    }

    private void MotorInit(DcMotorEx motor) {
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void ImuSetup() {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    private void MoveRobot() {
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

        front_left.setPower(frontLeftPower * speedVar);
        back_left.setPower(backLeftPower * speedVar);
        front_right.setPower(frontRightPower * speedVar);
        back_right.setPower(backRightPower * speedVar);

        RunIntake();
    }

    private void InitiateTransfer() {
        //outtake_elbow.setTargetPosition(outtakeMotorTransfer);
        //outtake_elbow.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //outtake_elbow.setPower(slidePower);
        outtake_wrist.setPosition(outtakeServoTransfer);

        target = intakeMotorTransfer;

        right_intake.setPosition(intakeServoTransfer);
        //left_intake.setPosition(intakeServoTransfer);
    }

    private void ResetTransfer1() {
        outtake_wrist.setPosition(outtakeServoDrop);

        target = resetVar1;

        right_intake.setPosition(intakeServoStart);

        target = resetVar2;

    }

    private void ResetTransfer2() {
        target = intakeMotorStart;
    }

    private void AdjustDown() {
        target -= 1;
    }

    private void AdjustUp() {
        target += 1;
    }

    private void NoDrop() {
        outtake_wrist.setPosition(outtakeServoDrop);
    }

    private void ManualSlidePos() {
        outtake_elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake_elbow.setPower(-gamepad1.right_stick_y);
    }

    private void TelemetryData() {
        double outtakePos = outtake_wrist.getPosition();
        double gameInput = gamepad1.right_stick_y;
        int intakePos = intake_elbow.getCurrentPosition();
        telemetry.addData("Intake Elbow Position", intakePos);
        telemetry.addData("Right Stick X", gamepad1.right_stick_x);
        telemetry.addData("Right Stick Y", gamepad1.right_stick_y);
        telemetry.addData("Outtake Servo Position", outtakePos);
        telemetry.addData("intake arm power", intakeArmPower);
        telemetry.addData("pid", pid);
        telemetry.addData("ff", ff);
        telemetry.addData("target arm angle", targetArmAngle);
        telemetry.addData("current arm angle", currentArmAngle);
        telemetry.addData("target", target);
        telemetry.update();
    }

    private void RunIntake() {
        if (gamepad1.left_bumper) {
            intake_grabber.setPower(1);
        } else if (gamepad1.right_bumper) {
            intake_grabber.setPower(-1);
        } else {
            intake_grabber.setPower(0);
        }
    }
}

