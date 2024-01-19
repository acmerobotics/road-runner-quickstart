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
    public static int intakeMotorTransfer1 = -80;
    public static int intakeMotorTransfer2 = -118;
    public static double speedVar = 1;
    public static int resetVar1 = -90;
    public static int resetVar2 = -40;
    public static int adjustSize = 5;
    IMU imu;
    private ElapsedTime runtime = new ElapsedTime();
    public enum ResetState {
        RESET_START,
        RESET_EXTEND,
        RESET_FALL,
        RESET_END,
        RESET_RESET1,
        RESET_RESET2
    };
    ResetState resetState = ResetState.RESET_START;
    public enum InitState {
        INIT_START,
        INIT_EXTEND,
        INIT_END
    };
    InitState initState = InitState.INIT_START;
    public boolean glideMode = false;
    public boolean slowMode = false;
    public boolean yToggle = false;
    public boolean isFieldCentric = true;

    private PIDController controller;
    public static double p = 0.02, i = 0, d = 0.0002;
    public static double f = -0.15;
    private final double ticks_in_degree = 144.0 / 180.0;
    public static int target;
    public static double offset = -25;
    int armPos;
    double pid, targetArmAngle, ff, currentArmAngle, intakeArmPower;
    Gamepad currentGamepad1;
    Gamepad previousGamepad1;
    @Override
    public void runOpMode() throws InterruptedException //START HERE
    {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        HardwareSetupMotors();
        HardwareSetupServos();
        ImuSetup();
        right_intake.setPosition(intakeServoStart);

        runtime.reset();
        target = 0;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            //Finite State Machine - Reset
            switch (resetState) {
                case RESET_START:
                    if (gamepad1.b) {
                        glideMode = false;
                        target = resetVar1;
                        resetState = ResetState.RESET_EXTEND;
                    }
                    break;
                case RESET_EXTEND:
                    if (Math.abs(intake_elbow.getCurrentPosition() - resetVar1) < 10) {
                        right_intake.setPosition(intakeServoStart);
                        outtake_wrist.setPosition(outtakeServoDrop);

                        runtime.reset();
                        resetState = ResetState.RESET_FALL;
                    }
                    break;
                case RESET_FALL:
                    if (runtime.seconds() >= .4) {
                        runtime.reset();
                        target = resetVar2;
                        resetState = ResetState.RESET_END;
                    }
                    break;
                case RESET_END:
                    if (runtime.seconds() >= .4) {

                        runtime.reset();
                        glideMode = true;
                        target = 0;

                        resetState = ResetState.RESET_RESET1;
                    }
                    break;
                case RESET_RESET1:
                    if (runtime.seconds() >= .8)
                    {
                        intake_elbow.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

                        resetState = ResetState.RESET_RESET2;
                    }
                    break;
                case RESET_RESET2:
                    if (runtime.seconds() >= .4)
                    {
                        intake_elbow.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

                        resetState = ResetState.RESET_START;
                    }
                default:
                    // should never be reached, as resetState should never be null
                    resetState = ResetState.RESET_START;
            }

            //Finite State Machine - Init
            switch (initState) {
                case INIT_START:
                    if (gamepad1.a) {
                        glideMode = false;
                        outtake_wrist.setPosition(outtakeServoTransfer);
                        initState = InitState.INIT_EXTEND;
                    }
                    break;
                case INIT_EXTEND:
                    if (runtime.seconds() >= .4) {

                        target = intakeMotorTransfer1;
                        right_intake.setPosition(intakeServoTransfer);
                        initState = InitState.INIT_END;
                    }
                    break;
                case INIT_END:
                    if (runtime.seconds() >= .4)
                    {
                        target = intakeMotorTransfer2;

                        initState = InitState.INIT_START;
                    }
                default:
                    initState = InitState.INIT_START;
            }

            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                AdjustDown();
            }
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                AdjustUp();
            }
            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
                outtake_wrist.setPosition(outtakeServoDrop);
            }
            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
                outtake_wrist.setPosition(outtakeServoTransfer);
            }
            if (currentGamepad1.back && !previousGamepad1.back) {
                glideMode = !glideMode;
            }
            if (currentGamepad1.left_stick_button && !previousGamepad1.left_stick_button)
            {
                isFieldCentric = !isFieldCentric;
            }
            if (currentGamepad1.x && !previousGamepad1.x) {
                intake_elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                intake_elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (currentGamepad1.y && !previousGamepad1.y)
            {
                glideMode = false;
                if (!yToggle)
                {
                    yToggle = true;
                    target -= 20;
                }
                else
                {
                    yToggle = false;
                    target = 0;
                    glideMode = true;
                }
            }
            //if (currentGamepad1.left_stick_button && currentGamepad1.right_stick_button) {
                //drone_launcher.setPosition();
            //}
            if (currentGamepad1.start && !previousGamepad1.start) {
                if (slowMode)
                {
                    slowMode = false;
                    speedVar = 1;
                }
                else
                {
                    slowMode = true;
                    speedVar = .6;
                }
            }

            if (isFieldCentric)
            {
                MoveRobotFieldCentric();
            }
            else
            {
                MoveRobot();
            }
            ManualSlidePos();
            RunIntake();

            //PID stuff
            controller.setPID(p, i, d);
            armPos = intake_elbow.getCurrentPosition();
            pid = controller.calculate(armPos, target);
            targetArmAngle = Math.toRadians((target - offset) / ticks_in_degree);
            ff = Math.cos(targetArmAngle) * f;
            currentArmAngle = Math.toRadians((armPos - offset) / ticks_in_degree);

            intakeArmPower = pid + ff;

            if (!glideMode)
            {
                intake_elbow.setPower(intakeArmPower);
                hang_arm.setPower(intakeArmPower);
            }
            else
            {
                intake_elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                hang_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                intake_elbow.setPower(0);
                hang_arm.setPower(0);
            }

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

        intake_elbow.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake_elbow.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
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

    private void MoveRobotFieldCentric() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (gamepad1.right_stick_button) {
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
    }
    private void MoveRobot()
    {
        double max;
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send calculated power to wheels
        front_left.setPower(speedVar * leftFrontPower);
        front_right.setPower(speedVar * rightFrontPower);
        back_left.setPower(speedVar * leftBackPower);
        back_right.setPower(speedVar * rightBackPower);
    }
    private void AdjustDown() {
        glideMode = false;
        target -= adjustSize;
    }

    private void AdjustUp() {
        glideMode = false;
        target += adjustSize;
    }
    private void ManualSlidePos()
    {
        outtake_elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake_elbow.setPower(-gamepad1.right_stick_y);
    }

    private void TelemetryData() {
        double outtakePos = outtake_wrist.getPosition();
        double gameInput = gamepad1.right_stick_y;
        int intakePos = intake_elbow.getCurrentPosition();
        telemetry.addData("Intake Elbow Position:", intakePos);
        telemetry.addData("Outtake Servo Position:", outtakePos);
        //telemetry.addData("intake arm power", intakeArmPower);
        //telemetry.addData("pid", pid);
        //telemetry.addData("ff", ff);
        //telemetry.addData("target arm angle", targetArmAngle);
        //telemetry.addData("current arm angle", currentArmAngle);
        telemetry.addData("Target:", target);
        telemetry.addData("GlideMode:", glideMode);
        telemetry.addData("SlowMode:", slowMode);
        telemetry.addData("Previous Input:", previousGamepad1);
        telemetry.addData("Current Input:", currentGamepad1);
        telemetry.addData("Run Time", runtime.seconds());
        telemetry.addData("Reset State", resetState.name());
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

