package org.firstinspires.ftc.teamcode.opModes.tests;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Robot_V2;
import org.firstinspires.ftc.teamcode.subsystems.multiaxisarm.MultiAxisArm;
import org.firstinspires.ftc.teamcode.subsystems.Slides;

@TeleOp(name="SystemsCheck", group="AAA_COMPETITION")
public class SystemsCheck extends OpMode {

    Pivot pivot = new Pivot();
    Slides slides = new Slides();

    AIMPad aimPad1;
    AIMPad aimPad2;

    enum TestingState {
        SLIDES, PIVOT
    }

    TestingState activeTestingState = TestingState.SLIDES;

    @Override
    public void init() {
        pivot.init(hardwareMap);
        slides.init(hardwareMap);

        aimPad1 = new AIMPad(gamepad1);
        aimPad2 = new AIMPad(gamepad2);
    }

    @Override
    public void loop() {
        aimPad1.update(gamepad1);
        aimPad2.update(gamepad2);

        switch (activeTestingState) {
            case SLIDES:
                slidesTest();
                break;
            case PIVOT:
                pivotTest();
                break;
        }

        telemetry.addData("Previous State", aimPad1.getPreviousState());
        telemetry.addData("Current State", aimPad1.getCurrentState());
        telemetry.addData("Current Testing State", activeTestingState);
        telemetry.update();
    }

    public void slidesTest() {
        slides.loop(aimPad1, aimPad2);
        slides.telemetry(telemetry);
        if (aimPad1.isAPressed()) {
            slides.setSlidesPosition(Slides.SlidesPosition.LOW_BUCKET);
        } else if (aimPad1.isBPressed()) {
            slides.setSlidesPosition(Slides.SlidesPosition.HIGH_BUCKET);
        } else if (aimPad1.isXPressed()) {
            slides.setSlidesPosition(Slides.SlidesPosition.RESET);
        }

        if (aimPad1.isYHeld()) {
            slides.setSlidesAtPower(-aimPad1.getRightStickY());
        }
        if (aimPad1.isStartPressed()) {
            activeTestingState = TestingState.PIVOT;
        }
    }

    public void pivotTest() {
        pivot.loop(aimPad1, aimPad2);
        pivot.telemetry(telemetry);
        if (aimPad1.isAPressed()) {
            pivot.setPivotPosition(Pivot.PivotPosition.PICKUP);
        } else if (aimPad1.isBPressed()) {
            pivot.setPivotPosition(Pivot.PivotPosition.SCORE);
        } else if (aimPad1.isXPressed()) {
            pivot.setPivotPosition(Pivot.PivotPosition.HANG);
        }

        if (aimPad1.isYHeld()) {
            pivot.setPivotAtPower(-aimPad1.getRightStickY());
        }
        if (aimPad1.isStartPressed()) {
            activeTestingState = TestingState.SLIDES;
        }
    }
}