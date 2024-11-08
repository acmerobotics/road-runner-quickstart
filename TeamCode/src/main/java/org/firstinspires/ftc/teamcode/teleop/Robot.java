package org.firstinspires.ftc.teamcode.teleop;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
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
    DcMotor leftFront, leftBack, rightFront, rightBack, leftHang, rightHang;
    DcMotor flip, slide;
    Servo wrist;
    CRServo intakeLeft, intakeRight;
    MecanumDrive drive;
    AnalogInput axonLeft, axonRight;
    PIDController armController, slideController;

    public final double gripClawOpen = 0, gripClawClosed = 0.1;
    public double intakeLeftPos, intakeRightPos;
    public double flipPos, slidePos;
    public int armTarget = 0, slideTarget = 0;

    public Robot(HardwareMap hardwareMap) {
//        axonLeft = hardwareMap.get(AnalogInput.class, "axonLeft");
//        axonRight = hardwareMap.get(AnalogInput.class, "axonRight");

//        intakeLeftPos = axonLeft.getVoltage() / 3.3 * 360;
//        intakeRightPos = axonRight.getVoltage() / 3.3 * 360;

        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        flip = hardwareMap.dcMotor.get("flip");
        slide = hardwareMap.dcMotor.get("slide");
        leftHang = hardwareMap.dcMotor.get("leftHang");
        rightHang = hardwareMap.dcMotor.get("rightHang");

        intakeLeft = hardwareMap.crservo.get("intakeLeft");
        intakeRight = hardwareMap.crservo.get("intakeRight");
        wrist = hardwareMap.servo.get("wrist");

        List<DcMotor> motors = Arrays.asList(leftBack, leftFront, rightBack, rightFront, flip, slide);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        flip.setDirection(DcMotorSimple.Direction.FORWARD);
        flip.setDirection(DcMotorSimple.Direction.FORWARD);

        leftHang.setDirection(DcMotorSimple.Direction.FORWARD);
        rightHang.setDirection(DcMotorSimple.Direction.REVERSE);


        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor motor: motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armController = new PIDController(armPIDValues.fP, armPIDValues.fI, armPIDValues.fD);
        slideController = new PIDController(armPIDValues.sP,armPIDValues.sI,armPIDValues.sD);
    }


    public void setTelemToDashboard(Telemetry telem) {
        telem = new MultipleTelemetry(telem, FtcDashboard.getInstance().getTelemetry());
    }

    public void arcadeDrive(Gamepad gamepad1) {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = 0.75*gamepad1.right_stick_x;

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
            y = -0.5*gamepad.left_stick_y;
            x = 0.5*gamepad.left_stick_x;
            rx = 0.5*gamepad.right_stick_x;
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
    public void arcadeDriveWithSlowModeForLittleChildren(Gamepad gamepad) {
        double y,x,rx;
        if (gamepad.right_trigger > 0) {
            y = -0.5*gamepad.left_stick_y;
            x = 0.5*gamepad.left_stick_x;
            rx = 0.5*gamepad.right_stick_x;
        }
        else {
            y = -0.4*gamepad.left_stick_y;
            x = 0.4*gamepad.left_stick_x;
            rx = 0.2*gamepad.right_stick_x;
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

//    public void tiltControl(Gamepad gamepad) {
//        flip.setPower(-gamepad.right_stick_y * 0.25);
//        if (gamepad.a) {
//            flip.setPower(1);
//        }
//        else if (gamepad.b) flip.setPower(-1);
//        else flip.setPower(0);
//    }

    public void wristControl(Gamepad gamepad) {
        if (gamepad.dpad_up) {
            wrist.setPosition(0.92);
        }
        else if (gamepad.dpad_down) {
            wrist.setPosition(0);
        }
        else if (gamepad.dpad_right) {
            wrist.setPosition(0.5);
        }
    }

    public void intakeControl(Gamepad gamepad) {
        intakeRight.setPower((-gamepad.left_trigger * 0.5 + gamepad.right_trigger));
        intakeLeft.setPower(gamepad.left_trigger - gamepad.right_trigger * 0.5);
    }

    public void hangControl(Gamepad gamepad) {
        if (gamepad.x)
        {
            leftHang.setPower(-1);
            rightHang.setPower(1);
        }
        else if (gamepad.b)
        {
            leftHang.setPower(1);
            rightHang.setPower(-1);
        }
        else {
            leftHang.setPower(0);
            rightHang.setPower(0);
        }
    }

    public void intakeOpen(Gamepad gamepad) {
//        if (gamepad.dpad_right) {
//            intakeRight.setPosition(intakeRight.getPosition() + 0.05);
//        }
//        else if (gamepad.dpad_left) {
//            intakeRight.setPosition(intakeRight.getPosition() - 0.05);
//        }
    }

//    public void updateAxonPositions() {
//        intakeLeftPos = axonLeft.getVoltage() / 3.3 * 360;
//        intakeRightPos = axonRight.getVoltage() / 3.3 * 360;
//    }

    public void scoringMacro(Gamepad gamepad1, Gamepad gamepad2) {
        if (gamepad2.y) {
            armTarget = 2200;
            while (Math.abs(armTarget - flip.getCurrentPosition()) > 1700) {
                TeleopPID(gamepad2);
                arcadeDrive(gamepad1);
            }
            slideTarget = 3000;
        }
        if (gamepad2.a) {
            slideTarget = 0;
            while (Math.abs(slideTarget - slide.getCurrentPosition()) > 1500) {
                TeleopPID(gamepad2);
                arcadeDrive(gamepad1);
            }
            armTarget = 0;
        }
        if (gamepad2.b) {
            slideTarget = 1200;
            wrist.setPosition(0);
        }
        else if (gamepad2.x) {
            slideTarget = 0;
            wrist.setPosition(0.5);
        }
    }

    public void TeleopPID(Gamepad gamepad) {
        armTarget += (int) ((int) -gamepad.right_stick_y * 20);
        slideTarget += (int) -gamepad.left_stick_y * 28;

        if (armTarget < 0) armTarget = 0;
        else if (armTarget > 2500) armTarget = 2500;

        if (slideTarget < 0) slideTarget = 0;
        else if (slideTarget > 3950) slideTarget = 3950;

        flipPos = flip.getCurrentPosition();
        slidePos = slide.getCurrentPosition();

        double pid = armController.calculate(flipPos, armTarget);
        double ff = Math.cos(Math.toRadians(armTarget / armPIDValues.ticks_in_degree)) * armPIDValues.fF;

        double power = pid + ff;

        flip.setPower(power);

        double pid2 = slideController.calculate(slidePos, slideTarget);

        slide.setPower(pid2);
    }
    public void slidesPID(Gamepad gamepad) {
//        double ff = Math.cos(Math.toRadians(armTarget / armPIDValues.ticks_in_degree)) * armPIDValues.fF;
//        flip.setPower((-gamepad.right_stick_y * 0.25) + ff);

        slideTarget += (int) -gamepad.left_stick_y * 28;
        if (slideTarget < 0) slideTarget = 0;
        else if (slideTarget > 5000) slideTarget = 5000;
        slidePos = slide.getCurrentPosition();

        double pid2 = slideController.calculate(slidePos, slideTarget);

        slide.setPower(pid2);
    }
    public void extraD1Features(Gamepad gamepad) {
        if (gamepad.dpad_up) {
            slideTarget += 28;
        }
        else if (gamepad.dpad_down) {
            slideTarget -= 28;
        }
        else if (gamepad.dpad_right) {
            armTarget += 15;
        }
        else if (gamepad.dpad_left) {
            armTarget -= 15;
        }

        intakeRight.setPower((-gamepad.left_trigger * 0.5 + gamepad.right_trigger));
        intakeLeft.setPower(gamepad.left_trigger - gamepad.right_trigger * 0.5);

        if (gamepad.y) {
            wrist.setPosition(0.92);
        }
        else if (gamepad.left_bumper) {
            wrist.setPosition(0);
        }
        else if (gamepad.right_bumper) {
            wrist.setPosition(0.5);
        }
    }

    public static class armPIDValues {
        public static double fP = 0.002, fI = 0.0002, fD = 0.0001;  //fD = 0.00001, fP = 0.002
        public static double fF = 0.005;
        public static double sP = 0.005, sI, sD;

        private static final double ticks_in_degree = 2048 / 90.0;
    }
    //4000, 2000
}

