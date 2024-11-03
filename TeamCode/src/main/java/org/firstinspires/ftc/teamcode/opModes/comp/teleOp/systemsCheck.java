package org.firstinspires.ftc.teamcode.opModes.comp.teleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.PAL;
import org.firstinspires.ftc.teamcode.subsystems.PIDSlides;
import org.firstinspires.ftc.teamcode.subsystems.settings.ConfigInfo;
import org.firstinspires.ftc.teamcode.subsystems.settings.ConfigurationInfo;
import org.firstinspires.ftc.teamcode.subsystems.settings.GamepadSettings;

@TeleOp(name="SystemsCheck", group="AAA_COMPETITION")
public class SystemsCheck extends OpMode {

    Drivebase drivebase = new Drivebase();
    Intake intake = new Intake();
    IntakeSystem intakeSystem = new IntakeSystem();
    Outtake outtake = new Outtake();
    OuttakeSystem outtakeSystem = new OuttakeSystem();
//    PIDSlides slides = new PIDSlides();
//    PAL pal = new PAL();

    enum TestingState {
        DRIVEBASE, INTAKE, INTAKE_SYSTEM, OUTTAKE, OUTTAKE_SYSTEM, ODOMETRY, CLAW, ARM, SLIDES, PAL
    }

    TestingState activeTestingState = TestingState.DRIVEBASE;
    @Override
    public void init() {
        odometry = new Odometry(hardwareMap);
        drivebase.init(hardwareMap);
        intake.init(hardwareMap);
        intakeSystem.init(hardwareMap);
        intake.init(hardwareMap);
        intake.init(hardwareMap);

        odometry.init(hardwareMap);
        claw.init(hardwareMap);
        arm.init(hardwareMap);
        slides.init(hardwareMap);
        pal.init(hardwareMap);
    }

    @Override
    public void loop() {
        // Stage 1: Individual system checks
        telemetry.addData("Stage 1: Individual System Checks", "Performing Individual System Checks");
        switch (activeTestingState) {
            case DRIVEBASE:
                drivebaseTest();
                break;
            case ODOMETRY:
                odoTest();
                break;
            case CLAW:
                clawTest();
                break;
            case ARM:
                armTest();
                break;
            case SLIDES:
                slidesTest();
                break;
            case PAL:
                palTest();
                break;
        }
        telemetry.addData("Current Testing State", activeTestingState);
        telemetry.update();
    }


    public void drivebaseTest() {
        drivebase.systemsCheck(gamepad1, telemetry);
        if (gamepad1.back) {
            activeTestingState = TestingState.ODOMETRY;
        }
    }

    public void odoTest() {
        odometry.systemsCheck(gamepad1, telemetry);
        if (gamepad1.start) {
            activeTestingState = TestingState.CLAW;
        }
    }

    public void clawTest() {
        claw.systemsCheck(gamepad1, telemetry);
        if (gamepad1.back) {
            activeTestingState = TestingState.ARM;
        }
    }

    public void armTest() {
        arm.systemsCheck(gamepad1, telemetry);
        if (gamepad1.start) {
            activeTestingState = TestingState.SLIDES;
        }
    }

    public void slidesTest() {
        slides.systemsCheck(gamepad1, telemetry);
        if (gamepad1.back) {
            activeTestingState = TestingState.PAL;
        }
    }

    public void palTest() {
        pal.systemsCheck(gamepad1, telemetry);
        if (gamepad1.start) {
            activeTestingState = TestingState.DRIVEBASE;
        }
    }
}
