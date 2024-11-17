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
    public DcMotor leftFront, leftBack, rightFront, rightBack;
    public DcMotor flip, slide;
    public Servo wrist, leftHang, rightHang;
    public CRServo intakeLeft, intakeRight;
    public MecanumDrive drive;
    public PIDController armController, slideController;

    public final double gripClawOpen = 0, gripClawClosed = 0.1;
    public double flipPos, slidePos;
    public int armTarget = 0, slideTarget = 0;
    public int armTargetAuto = 0, slideTargetAuto = 0;
    public static volatile boolean stopPid = false;
    public double wristTargetAuto = 0.0;
    public double intakeMultiplier = 1;
    Thread currentThread = null;

    public Robot(HardwareMap hardwareMap) {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        flip = hardwareMap.dcMotor.get("flip");
        slide = hardwareMap.dcMotor.get("slide");
        leftHang = hardwareMap.servo.get("leftHang");
        rightHang = hardwareMap.servo.get("rightHang");

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
            wrist.setPosition(0.5);
        }
        else if (gamepad.dpad_down) {
            wrist.setPosition(0.07);
        }
        else if (gamepad.dpad_right) {
            wrist.setPosition(0.35);
        }
        else if (gamepad.dpad_left) {
            wrist.setPosition(0.95);
        }
    }

    public void intakeControl(Gamepad gamepad) {
        intakeRight.setPower(intakeMultiplier*(-gamepad.left_trigger + gamepad.right_trigger));
        intakeLeft.setPower(intakeMultiplier*(gamepad.left_trigger - gamepad.right_trigger));
    }

    public void hangControl(Gamepad gamepad) {
        if (gamepad.a)
        {
            leftHang.setPosition(1);
            rightHang.setPosition(-1);
        }
        else if (gamepad.y)
        {
            leftHang.setPosition(-1);
            rightHang.setPosition(0.93);
        }
    }


    public void scoringMacro(Gamepad gamepad1, Gamepad gamepad2) {
        if (gamepad2.y) {
            armTarget = 2200;
            wrist.setPosition(0.5);
            intakeMultiplier = 0.5;
            while (Math.abs(armTarget - flip.getCurrentPosition()) > 100) {
                TeleopPID(gamepad2);
                arcadeDrive(gamepad1);
            }
            slideTarget = 6000;
        }
        if (gamepad2.a) {
            slideTarget = 0;
            intakeMultiplier = 1;
            while (Math.abs(slideTarget - slide.getCurrentPosition()) > 3000) {
                TeleopPID(gamepad2);
                arcadeDrive(gamepad1);
            }
            armTarget = 0;
        }
        if (gamepad2.x) {
            slideTarget = 2000;
            intakeMultiplier = 1;
            while (Math.abs(slideTarget - slide.getCurrentPosition()) > 1500) {
                TeleopPID(gamepad2);
                arcadeDrive(gamepad1);
            }
            wrist.setPosition(0.07);
        }
        else if (gamepad2.b) {
            slideTarget = 0;
            intakeMultiplier = 1;
            wrist.setPosition(0.35);
        }
    }

    public void TeleopPID(Gamepad gamepad) {
        armTarget += (int) ((int) -gamepad.right_stick_y * 20);
        slideTarget += (int) -gamepad.left_stick_y * 28;

        if (armTarget < 0) armTarget = 0;
        else if (armTarget > 3000) armTarget = 3000;

        if (slideTarget < 0) slideTarget = 0;
        else if (armTarget < 100 && slideTarget > 2300) slideTarget = 2300;
        else if (slideTarget > 6000) slideTarget = 6000;

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

//        intakeRight.setPower((-gamepad.left_trigger + gamepad.right_trigger));
//        intakeLeft.setPower(gamepad.left_trigger - gamepad.right_trigger);

//        if (gamepad.left_trigger > 0 && !(gamepad.right_trigger > 0)) {
//            intakeLeft.setPower(1);
//            intakeRight.setPower(-1);
//        }
//        if (gamepad.right_trigger > 0 && !(gamepad.left_trigger > 0)) {
//            intakeLeft.setPower(-1);
//            intakeRight.setPower(1);
//        }

//        if (gamepad.y) {
//            wrist.setPosition(0.92);
//        }
//        else if (gamepad.left_bumper) {
//            wrist.setPosition(0);
//        }
//        else if (gamepad.right_bumper) {
//            wrist.setPosition(0.5);
//        }
    }
    public void startPID() {
        if (currentThread == null || !currentThread.isAlive()) {
            stopPid = false;
            Thread thread = new Thread(new pidfLoopAuton());
            currentThread = thread;
            currentThread.start();
        }
    }
    public void setIntakePower(double power) {
        intakeLeft.setPower(power);
        intakeRight.setPower(-power);
    }
    public Action intake(double power) {
        return new intakeAction(power);
    }
    public class intakeAction implements Action {
        double intakePower;

        public intakeAction(double power) {
            intakePower = power;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setIntakePower(intakePower);
            return false;
        }
    }
    public Action setPidVals(int arm, int slide) {
        return new ValAction(arm, slide);
    }
    public Action wrist(double wrist) {
        return new wristAction(wrist);
    }
    public class wristAction implements Action {
        double wristPos;
        public wristAction(double wrist) {
            wristPos = wrist;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(wristPos);
            return false;
        }
    }
    public Action getPIDAction() {
        return new pidfLoopAction();
    }
    public void stopPidAction() {
        stopPid = true;
    }
    public class pidfLoopAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            flipPos = flip.getCurrentPosition();
            slidePos = slide.getCurrentPosition();

            double pid = armController.calculate(flipPos, armTargetAuto);
            double ff = Math.cos(Math.toRadians(armTargetAuto / armPIDValues.ticks_in_degree)) * armPIDValues.fF;

            double power = pid + ff;

            flip.setPower(power);

            double pid2 = slideController.calculate(slidePos, slideTargetAuto);

            slide.setPower(pid2);

//            try {
//                Thread.sleep(10); // Adjust the sleep time as needed
//            } catch (InterruptedException e) {
//                Thread.currentThread().interrupt();
//            }
            return !stopPid;
        }
    }
    public class ValAction implements Action {
        int armTarget, slidetarget;
        public ValAction(int arm, int slide) {
            armTarget = arm;
            slidetarget = slide;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setPidValues(armTarget, slidetarget);
            return false;
        }
    }
    private class pidfLoopAuton implements Runnable {
        @Override
        public void run() {
            while (!stopPid) {
                flipPos = flip.getCurrentPosition();
                slidePos = slide.getCurrentPosition();

                double pid = armController.calculate(flipPos, armTargetAuto);
                double ff = Math.cos(Math.toRadians(armTargetAuto / armPIDValues.ticks_in_degree)) * armPIDValues.fF;

                double power = pid + ff;

                flip.setPower(power);

                double pid2 = slideController.calculate(slidePos, slideTargetAuto);

                slide.setPower(pid2);

                try {
                    Thread.sleep(10); // Adjust the sleep time as needed
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    }
    public Action stopPID() {
        return new stopPid();
    }
    public class stopPid implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Robot.stopPid = true;
            try {
                if (currentThread != null && currentThread.isAlive()) {
                    currentThread.join(); // Wait for the thread to finish
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
           // currentThread.stop();

            armTargetAuto = 0;
            slideTargetAuto = 0;
            flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            return false;
        }
    }
    public void setPidValues(int arm, int slide) {
        armTargetAuto = arm;
        slideTargetAuto = slide;
    }

    public static class armPIDValues {
        public static double fP = 0.0043, fI = 0.0015, fD = 0.0003;  //fD = 0.00001, fP = 0.002
        public static double fF = 0.04;  //fF = 0.0022
        public static double sP = 0.003, sI, sD;

        private static final double ticks_in_degree = 2048 / 90.0;
    }
    //4000, 2000
}

