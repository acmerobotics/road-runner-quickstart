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

    //all servo positioning stuff is from 0 - 1 (decimals included) and not in radians / degrees for some reason, 0 is 0 degrees, 1 is 320 (or whatever the servo max is) degrees
    //all our servos have 320 degrees of movement so i limited it so it wont collide with the arm too much
    private double clawMax = 0.7; //maximum angle the claw servo is allowed to move
    private double clawMin = 0.3; //minimum angle the claw servo is allowed to move

    private double clawYPos = (clawMax + clawMin) / 2; //uses this value to set the initial claw position in the middle of the max and min
    //i am using a variable because .getPosition() only returns the last position the servo was told to move, not its actual location


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

        clawUpDown.setPosition(clawYPos); //im setting the position of the servo here because we can't read the servos actual angle

    }

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");

        clawUpDown = hardwareMap.get(Servo.class, "servoUpDown"); //add a servo onto the robot just to make sure this works (idk if this will error without one)

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


        double clawIncrement = 0.05; //how much the claw angle increases / decreases every time the buttons are down

        y = gamepad1.left_stick_y;
        x = -gamepad1.left_stick_x;
        t = -gamepad1.right_stick_x;

        if (gamepad1.start) {
            imu.resetYaw();
        }

        if (gamepad1.dpad_left && !gamepad1.dpad_right) {
            //claw down
            clawYPos = clamp(clawYPos - clawIncrement, clawMin, clawMax);



        }else if (gamepad1.dpad_right && !gamepad1.dpad_left){
            //claw up
            clawYPos = clamp(clawYPos + clawIncrement, clawMin, clawMax);



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
        clawUpDown.setPosition(clawYPos); //set servo position

        telemetry.addData("claw angle: ", clawUpDown.getPosition());

    }
}