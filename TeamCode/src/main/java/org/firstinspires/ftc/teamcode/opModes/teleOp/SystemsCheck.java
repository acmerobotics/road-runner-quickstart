package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.PIDSlides;
import org.firstinspires.ftc.teamcode.subsystems.settings.GamepadSettings;

@TeleOp(name="SystemsCheck", group="AAA_COMPETITION")
public class SystemsCheck extends OpMode {
    Drivebase drivebase = new Drivebase(true);
    Odometry odometry = new Odometry();
    Camera camera = new Camera();
    Claw claw = new Claw();
    Arm arm = new Arm();
    Intake intake = new Intake();
    PIDSlides slides = new PIDSlides();

    enum TestingState {
        DRIVEBASE, ODOMETRY, CAMERA, CLAW, ARM, INTAKE, SLIDES
    }

    TestingState activeTestingState = TestingState.DRIVEBASE;

    @Override
    public void init() {
        drivebase.init(hardwareMap);
        odometry.init(hardwareMap);
        camera.init(hardwareMap);
        claw.init(hardwareMap);
        arm.init(hardwareMap);
        intake.init(hardwareMap);
        slides.init(hardwareMap);
    }

    @Override
    public void loop() {
        switch (activeTestingState) {
            case DRIVEBASE:
                drivebaseTest();
                break;
            case ODOMETRY:
                odoTest();
                break;
            case CAMERA:
                cameraTest();
                break;
            case CLAW:
                clawTest();
                break;
            case ARM:
                armTest();
                break;
            case INTAKE:
                intakeTest();
                break;
            case SLIDES:
                slidesTest();
                break;
        }
        telemetry.addData("Current Testing State", activeTestingState);
        telemetry.update();
    }


    public void drivebaseTest() {
        drivebase.loop(gamepad1);
        drivebase.telemetry(telemetry);
        if (gamepad1.a && gamepad2.a) {
            activeTestingState = TestingState.ODOMETRY;
        }
    }

    public void odoTest() {
        odometry.telemetry(telemetry);
        if (gamepad1.a && gamepad2.a) {
            activeTestingState = TestingState.CAMERA;
        }
    }

    public void cameraTest() {
        // TODO: Add camera test
        if (gamepad1.a && gamepad2.a) {
            activeTestingState = TestingState.CLAW;
        }
    }

    public void clawTest() {
        if (gamepad1.left_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE) {
            claw.clampServo(claw.leftProng);
        } else if (gamepad1.right_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE) {
            claw.clampServo(claw.rightProng);
        } else {
            claw.releaseServo(claw.leftProng);
            claw.releaseServo(claw.rightProng);
        }

        if (gamepad1.a) {
            claw.tilt(claw.tiltedLeftPos);
        } else if (gamepad1.b) {
            claw.tilt(claw.tiltedRightPos);
        } else {
            claw.tilt(claw.straight);
        }
        if (gamepad1.a && gamepad2.a) {
            activeTestingState = TestingState.ARM;
        }
    }

    public void armTest() {
        if (gamepad1.a) {
            arm.safeRetract();
        } else if (gamepad1.b) {
            arm.retract();
        } else if (gamepad1.y){
            arm.extend();
        }
        if (gamepad1.a && gamepad2.a) {
            activeTestingState = TestingState.INTAKE;
        }
    }

    public void intakeTest() {
        // TODO: Add intake test
        if (gamepad1.a && gamepad2.a) {
            activeTestingState = TestingState.SLIDES;
        }
    }

    public void slidesTest() {
        if (Math.abs(gamepad1.left_stick_y) > GamepadSettings.GP2_STICK_DEADZONE) {
            slides.setPower(gamepad1.left_stick_y);
        } else if (gamepad1.a) {
            slides.update(PIDSlides.SAFE_RETRACTION_POS);
        } else if (gamepad1.b) {
            slides.update(PIDSlides.SAFE_EXTENSION_POS);
        } else if (gamepad1.y) {
            slides.update(PIDSlides.RESET_POS);
        } else {
            slides.holdPosition();
        }
        if (gamepad1.a && gamepad2.a) {
            activeTestingState = TestingState.DRIVEBASE;
        }
    }
}
