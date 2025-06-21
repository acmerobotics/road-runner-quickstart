package org.firstinspires.ftc.teamcode.drive.opmode.manual;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="compOpMode", group="practice")
public class mainManual extends OpMode{

    // Create motor & servo variables
    DcMotorEx leftFront, rightFront, leftBack, rightBack; // Drive Motors
    DcMotorEx leftSlide, rightSlide, leftSlidePivot, rightSlidePivot; // Slide Motors

    // Servo leftClaw, rightClaw; // Claw Servos

    double leftSlideStartPos;
    double rightSlideStartPos;

    double leftSlidePivStartPos;
    double rightSlidePivStartPos;

// Create PID Instances for slides and pivots
//    private PIDController leftSlidePID;
//    private PIDController rightSlidePID;
//    private PIDController leftPivotPID;
//    private PIDController rightPivotPID;

    private ElapsedTime loopTimer;
    private final double targetLoopTime = 0.05;

    public void init() {
        loopTimer = new ElapsedTime();

//        leftSlidePID = new PIDController(0.0, 0.0, 0.0);
//        rightSlidePID = new PIDController(0.0, 0.0, 0.0);
//        leftPivotPID = new PIDController(0.0, 0.0, 0.0);
//        rightPivotPID = new PIDController(0.0, 0.0, 0.0);

        // Drive Motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Slide Motors
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftSlidePivot = hardwareMap.get(DcMotorEx.class, "leftSlidePivot");
        leftSlidePivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightSlidePivot = hardwareMap.get(DcMotorEx.class, "rightSlidePivot");
        rightSlidePivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Claw Servos
        // leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        // rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        // Set the slide motors to brake when no power is applied
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlidePivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlidePivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: Reverse Necessary Motors
        leftSlidePivot.setDirection(DcMotorSimple.Direction.REVERSE); // Reverse left pivot motor
        //leftClaw.setDirection(Servo.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        leftSlideStartPos = leftSlide.getCurrentPosition();
        rightSlideStartPos = rightSlide.getCurrentPosition();

        leftSlidePivStartPos = leftSlidePivot.getCurrentPosition();
        rightSlidePivStartPos = rightSlidePivot.getCurrentPosition();
    }

    public void loop() {
        // Measure elapsed time since last loop
        double elapsedTime = loopTimer.seconds();

        // If elapsedTime is less than target, wait until target time is reached
        while (elapsedTime < targetLoopTime) {
            elapsedTime = loopTimer.seconds();
        }

        // Reset the loop timer for the next cycle
        loopTimer.reset();

        // Gamepad 1 controls the robot's movement
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rotInput = -gamepad1.right_stick_x;

        leftFront.setPower((y + x - rotInput));
        leftBack.setPower((-y + x + rotInput)); // -y
        rightFront.setPower((y + x + rotInput));
        rightBack.setPower((y - x + rotInput));

        // Gamepad 2 controls the viperslides
        // Programming claw
        double closedPos = 0.65;
        double openPos = 0.42;
        double movementStep = 0.01; // Step size

        // Variables to track current claw positions
//        double leftClawPosition = leftClaw.getPosition();
//        double rightClawPosition = rightClaw.getPosition();
//
//        if (gamepad2.right_bumper) {
//            leftClawPosition = Math.min(leftClawPosition + movementStep, openPos);
//            rightClawPosition = Math.min(rightClawPosition + movementStep, openPos);
//        }
//        else {
//            leftClawPosition = Math.max(leftClawPosition - movementStep, closedPos);
//            rightClawPosition = Math.max(rightClawPosition - movementStep, closedPos);
//        }
//
//        leftClaw.setPosition(leftClawPosition);
//        rightClaw.setPosition(rightClawPosition);

        // Get stick input and reference velocity for slide control
        double stickInputR = -gamepad2.right_stick_y; // Right stick controls slides
        double maxSlideVelocity = 500;
        double slideRefVelocity = stickInputR * maxSlideVelocity;

        // Get stick input and reference velocity for pivot control
        double stickInputL = gamepad2.left_stick_y; // Left stick controls pivot
        double maxPivVelocity = 500;
        double pivRefVelocity = stickInputL * maxPivVelocity;

        // Apply dead zone around 0 - Don't want drift
        if (Math.abs(stickInputL) < 0.05) {
            pivRefVelocity = 0;
        }
        if (Math.abs(stickInputR) < 0.05) {
            slideRefVelocity = 0;
        }

//        // PID controls for slides
//        double slidePowerL = leftSlidePID.calculate(slideRefVelocity, leftSlide.getVelocity());
//        leftSlide.setPower(slidePowerL);
//
////        double slidePowerR = rightSlidePID.calculate(slideRefVelocity, rightSlide.getVelocity());
//        rightSlide.setPower(slidePowerL);
//
//        // PID controls for pivot
//        double pivotVelocityL = leftSlidePivot.getVelocity();
//        double pivPowerL = leftPivotPID.calculate(pivRefVelocity, pivotVelocityL);
//        leftSlidePivot.setPower(pivPowerL);
//
//        double pivotVelocityR = rightSlidePivot.getVelocity();
//        double pivPowerR = rightPivotPID.calculate(pivRefVelocity, pivotVelocityR);
//        rightSlidePivot.setPower(pivPowerR);

        double pivot = gamepad2.right_stick_y;

        leftSlidePivot.setPower(pivot);
        rightSlidePivot.setPower(-pivot);

        double vert = gamepad2.left_stick_y;

        leftSlide.setPower(-vert);
        rightSlide.setPower(vert);

//        telemetry.addData("Left Servo Position", leftClaw.getPosition());
//        telemetry.addData("Right Servo Position", rightClaw.getPosition());

        telemetry.addData("Left Slide Position", leftSlide.getCurrentPosition());
        telemetry.addData("Right Slide Position", rightSlide.getCurrentPosition());
        telemetry.addData("Left Pivot Position", leftSlidePivot.getCurrentPosition());
        telemetry.addData("Right Pivot Position", rightSlidePivot.getCurrentPosition());

        telemetry.addData("Left Slide Velocity", leftSlide.getVelocity());
        telemetry.addData("Right Slide Velocity", rightSlide.getVelocity());
        telemetry.addData("Left Pivot Velocity", leftSlidePivot.getVelocity());
        telemetry.addData("Right Pivot Velocity", rightSlidePivot.getVelocity());

        telemetry.addData("Left Stick Input", stickInputL);
        telemetry.addData("Right Stick Input", stickInputR);

//        telemetry.addData("Pivot Reference Velocity", pivRefVelocity);
//        telemetry.addData("Slide Reference Velocity", slideRefVelocity);
        telemetry.addData("Elapsed Time", elapsedTime);
        telemetry.addData("Loop Frequency (Hz)", 1.0 / elapsedTime);
        telemetry.update();
    }
}
