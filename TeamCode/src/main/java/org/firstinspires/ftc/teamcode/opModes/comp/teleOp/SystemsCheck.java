package org.firstinspires.ftc.teamcode.opModes.comp.teleOp;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.v1.Outdated_Intake;
import org.firstinspires.ftc.teamcode.subsystems.v1.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.v1.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.v1.OuttakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.v1.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.v1.SpecimenGrabber;

@TeleOp(name="SystemsCheck", group="AAA_COMPETITION")
public class SystemsCheck extends OpMode {

    Drivebase drivebase = new Drivebase();
    IntakeSystem intakeSystem = new IntakeSystem();
    Outtake outtake = new Outtake();
    OuttakeSystem outtakeSystem = new OuttakeSystem();
    SpecimenGrabber specimenGrabber = new SpecimenGrabber();

    AIMPad aimPad1;
    AIMPad aimPad2;


    enum TestingState {
        DRIVEBASE, INTAKE_SYSTEM, OUTTAKE, OUTTAKE_SYSTEM, SPECIMEN_GRABBER
    }

    TestingState activeTestingState = TestingState.DRIVEBASE;

    @Override
    public void init() {
        drivebase.init(hardwareMap);
        intakeSystem.init(hardwareMap);
        outtake.init(hardwareMap);
        outtakeSystem.init(hardwareMap);
        specimenGrabber.init(hardwareMap);

        aimPad1 = new AIMPad(gamepad1);
        aimPad2 = new AIMPad(gamepad2);
    }

    @Override
    public void loop() {
        aimPad1.update(gamepad1);
        aimPad2.update(gamepad2);

        switch (activeTestingState) {
            case DRIVEBASE:
                drivebaseTest();
                break;
            case INTAKE_SYSTEM:
                intakeSystemTest();
                break;
            case OUTTAKE:
                outtakeTest();
                break;
            case OUTTAKE_SYSTEM:
                outtakeSystemTest();
                break;
            case SPECIMEN_GRABBER:
                specimenGrabberTest();
                break;
        }
        telemetry.addData("Advance Pressed", aimPad1.isStartPressed());
        telemetry.addData("Advance Released", aimPad1.isStartReleased());
        telemetry.addData("Previous State", aimPad1.getPreviousState());
        telemetry.addData("Current State", aimPad1.getCurrentState());
        telemetry.addData("Current Testing State", activeTestingState);
        telemetry.update();
    }


    public void drivebaseTest() {
        drivebase.systemsCheck(aimPad1, telemetry);
        if (aimPad1.isStartPressed()) {
            activeTestingState = TestingState.INTAKE_SYSTEM;
        }
    }

    public void intakeSystemTest() {
        intakeSystem.systemsCheck(aimPad1, aimPad2, telemetry);
        if (aimPad1.isStartPressed()) {
            activeTestingState = TestingState.OUTTAKE;
        }
    }

    public void outtakeTest() {
        outtake.systemsCheck(aimPad1, telemetry);
        if (aimPad1.isStartPressed()) {
            activeTestingState = TestingState.OUTTAKE_SYSTEM;
        }
    }

    public void outtakeSystemTest() {
        outtakeSystem.systemsCheck(aimPad1, aimPad2, telemetry);
        if (aimPad1.isStartPressed()) {
            activeTestingState = TestingState.SPECIMEN_GRABBER;
        }
    }

    public void specimenGrabberTest() {
        specimenGrabber.systemsCheck(aimPad1, telemetry);
        if (aimPad1.isStartPressed()) {
            activeTestingState = TestingState.DRIVEBASE;
        }
    }
}