package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Arrays;
import java.util.List;

//TODO: change claw opened and closed values
public class Robot {
    DcMotor leftFront, leftBack, rightFront, rightBack;
    DcMotor flip, slide;
    Servo wrist;
    Servo claw;
//    Servo intakeRight;
    CRServo intakeLeft, intakeRight;
    MecanumDrive drive;
    AnalogInput axonLeft, axonRight;

    public final double gripClawOpen = 0, gripClawClosed = 0.1;
    public double intakeLeftPos, intakeRightPos;

    public Robot(HardwareMap hardwareMap) {
        axonLeft = hardwareMap.get(AnalogInput.class, "axonLeft");
        axonRight = hardwareMap.get(AnalogInput.class, "axonRight");

        intakeLeftPos = axonLeft.getVoltage() / 3.3 * 360;
        intakeRightPos = axonRight.getVoltage() / 3.3 * 360;

        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        flip = hardwareMap.dcMotor.get("flip");
        slide = hardwareMap.dcMotor.get("slide");

        intakeLeft = hardwareMap.crservo.get("intakeLeft");
        intakeRight = hardwareMap.crservo.get("intakeRight");
        claw = hardwareMap.servo.get("claw");
        wrist = hardwareMap.servo.get("wrist");

        List<DcMotor> motors = Arrays.asList(leftBack, leftFront, rightBack, rightFront, flip, slide);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        flip.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor motor: motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public Action pidfLoopFlip(double target) {
        pidfLoop loop = new pidfLoop();
        return loop.getPivotLoop(flip, target);
    }

    public Action pidfLoopSlides(double target) {
        pidfLoop loop = new pidfLoop();
        return loop.getSlidesLoop(slide, target);
    }

    public void setTelemToDashboard(Telemetry telem) {
        telem = new MultipleTelemetry(telem, FtcDashboard.getInstance().getTelemetry());
    }

    public void arcadeDrive(Gamepad gamepad1) {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator;
        double leftBackPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightBackPower = (y + x - rx) / denominator;

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }

    public void arcadeDriveWithSlowMode(Gamepad gamepad) {
        double y,x,rx;
        if (gamepad.right_trigger > 0) {
            y = -0.2*gamepad.left_stick_y;
            x = 0.2*gamepad.left_stick_x;
            rx = 0.2*gamepad.right_stick_x;
        }
        else {
            y = -gamepad.left_stick_y;
            x = gamepad.left_stick_x;
            rx = 0.75*gamepad.right_stick_x;
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator;
        double leftBackPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightBackPower = (y + x - rx) / denominator;

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }

    public void slideControl(Gamepad gamepad) {
        slide.setPower(-gamepad.left_stick_y * 0.3);
    }

    public void tiltControl(Gamepad gamepad) {
        flip.setPower(-gamepad.right_stick_y * 0.25);
    }

    public void wristControl(Gamepad gamepad) {
        if (gamepad.dpad_up) {
            wrist.setPosition(wrist.getPosition() + 0.05);
        }
        else if (gamepad.dpad_down) {
            wrist.setPosition(wrist.getPosition() - 0.05);
        }
    }

    public void gripClawControl(Gamepad gamepad) {
        if (gamepad.a) {
            claw.setPosition(gripClawClosed);
        }
        else if (gamepad.b) {
            claw.setPosition(gripClawOpen);
        }
    }

    public void intakeControl(Gamepad gamepad) {
        intakeLeft.setPower(-gamepad.left_trigger + gamepad.right_trigger);
         intakeRight.setPower(gamepad.left_trigger - gamepad.right_trigger);
    }

    public void intakeOpen(Gamepad gamepad) {
//        if (gamepad.dpad_right) {
//            intakeRight.setPosition(intakeRight.getPosition() + 0.05);
//        }
//        else if (gamepad.dpad_left) {
//            intakeRight.setPosition(intakeRight.getPosition() - 0.05);
//        }
    }

    public void updateAxonPositions() {
        intakeLeftPos = axonLeft.getVoltage() / 3.3 * 360;
        intakeRightPos = axonRight.getVoltage() / 3.3 * 360;
    }
}
