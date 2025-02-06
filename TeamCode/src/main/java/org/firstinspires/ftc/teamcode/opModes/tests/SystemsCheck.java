package org.firstinspires.ftc.teamcode.opModes.tests;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.ScoringAssembly;
import org.firstinspires.ftc.teamcode.subsystems.Slides;

@TeleOp(name="SystemsCheck", group="AAA_COMPETITION")
public class SystemsCheck extends OpMode {

    ScoringAssembly scoringAssembly = new ScoringAssembly();
    AIMPad aimPad1;
    AIMPad aimPad2;

    enum TestingState {
        SLIDES, PIVOT, ARM
    }

    TestingState activeTestingState = TestingState.SLIDES;

    @Override
    public void init() {
        scoringAssembly.init(hardwareMap);

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
            case ARM:
                armTest();
                break;
        }

        telemetry.addData("Previous State", aimPad1.getPreviousState());
        telemetry.addData("Current State", aimPad1.getCurrentState());
        telemetry.addData("Current Testing State", activeTestingState);
        telemetry.update();
    }

    public void slidesTest() {
        scoringAssembly.slides.loop(aimPad1, aimPad2);
        scoringAssembly.slides.telemetry(telemetry);
        if (aimPad1.isAPressed()) {
            scoringAssembly.slides.setSlidesPosition(Slides.SlidesExtension.LOW_BUCKET);
        } else if (aimPad1.isBPressed()) {
            scoringAssembly.slides.setSlidesPosition(Slides.SlidesExtension.HIGH_BUCKET);
        } else if (aimPad1.isXPressed()) {
            scoringAssembly.slides.setSlidesPosition(Slides.SlidesExtension.RESET);
        }

        if (aimPad1.isYHeld()) {
            scoringAssembly.slides.setSlidesAtPower(-aimPad1.getRightStickY());
        }
        if (aimPad1.isStartPressed()) {
            activeTestingState = TestingState.PIVOT;
        }
    }

    public void pivotTest() {
        scoringAssembly.pivot.loop(aimPad1, aimPad2);
        scoringAssembly.pivot.telemetry(telemetry);
        if (aimPad1.isAPressed()) {
            scoringAssembly.pivot.setPivotPosition(Pivot.PivotAngle.LOW_HANG);
        } else if (aimPad1.isBPressed()) {
            scoringAssembly.pivot.setPivotPosition(Pivot.PivotAngle.SCORE);
        } else if (aimPad1.isXPressed()) {
            scoringAssembly.pivot.setPivotPosition(Pivot.PivotAngle.PICKUP);
        }

        if (aimPad1.isYHeld()) {
            scoringAssembly.pivot.setPivotAtPower(-aimPad1.getRightStickY());
        }
        if (aimPad1.isStartPressed()) {
            activeTestingState = TestingState.SLIDES;
        }
    }

    public void armTest() {
        scoringAssembly.multiAxisArm.loop(aimPad1, aimPad2);
    }
}