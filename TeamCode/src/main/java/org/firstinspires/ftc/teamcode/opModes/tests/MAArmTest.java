package org.firstinspires.ftc.teamcode.opModes.tests;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.multiaxisarm.MultiAxisArm;
import org.firstinspires.ftc.teamcode.subsystems.Slides;

@TeleOp(name="ArmTest", group="AAA_COMPETITION")
public class MAArmTest extends OpMode {

    MultiAxisArm arm = new MultiAxisArm();
    Slides slides = new Slides();

    AIMPad aimPad1;
    AIMPad aimPad2;

    enum TestingState {
        HAND, FLEXOR, ROTATOR, ELBOW, FULL
    }

    TestingState activeTestingState = TestingState.HAND;

    @Override
    public void init() {
        arm.init(hardwareMap);
        slides.init(hardwareMap);

        aimPad1 = new AIMPad(gamepad1);
        aimPad2 = new AIMPad(gamepad2);
    }

    @Override
    public void loop() {
        aimPad1.update(gamepad1);
        aimPad2.update(gamepad2);

        switch (activeTestingState) {
            case HAND:
                handTest();
                break;
            case FLEXOR:
                flexorTest();
                break;
            case ROTATOR:
                rotatorTest();
                break;
            case ELBOW:
                elbowTest();
                break;
            case FULL:
                fullTest();
                break;
        }
        telemetry.addData("Advance Pressed", aimPad1.isStartPressed());
        telemetry.addData("Advance Released", aimPad1.isStartReleased());
        telemetry.addData("Previous State", aimPad1.getPreviousState());
        telemetry.addData("Current State", aimPad1.getCurrentState());
        telemetry.addData("Current Testing State", activeTestingState);
        telemetry.update();
    }

    public void handTest() {
        arm.hand.hand.systemsCheck(aimPad1, telemetry);
        if (aimPad1.isStartPressed()) {
            activeTestingState = TestingState.FLEXOR;
        }
    }

    public void flexorTest() {
        arm.wrist.flexor.systemsCheck(aimPad1, telemetry);
        if (aimPad1.isStartPressed()) {
            activeTestingState = TestingState.ROTATOR;
        }
    }

    public void rotatorTest() {
        arm.wrist.rotator.systemsCheck(aimPad1, telemetry);
        if (aimPad1.isStartPressed()) {
            activeTestingState = TestingState.ELBOW;
        }
    }

    public void elbowTest() {
        arm.elbow.leftElbow.systemsCheck(aimPad1, telemetry);
        if (aimPad1.isStartPressed()) {
            activeTestingState = TestingState.FULL;
        }
    }

    public void fullTest() {
        arm.loop(aimPad1);
        if (aimPad1.isRightTriggerPressed()) {
            //TODO potentially swap for tested aimPad method (>0.5)
            arm.hand.toggle();
            //ADD CODE TO TOGGLE CLASS in hand FILE (to toggle open and close)
        }
        if (aimPad1.isRightStickMovementHeld()) {
            double rightStickX = aimPad1.getRightStickX();

            //Controls LEFT and RIGHT, change values as needed
            if (rightStickX > 0.5) {
                arm.wrist.rotateRight();
            } else if (rightStickX < -0.5) {
                arm.wrist.rotateLeft();
            } else {
                arm.wrist.rotateCenter();
            }
        }
        if (aimPad1.isYPressed()){
            arm.wrist.flexUp();
        } else if (aimPad1.isAPressed()){
            arm.wrist.flexDown();
        }
    }
}