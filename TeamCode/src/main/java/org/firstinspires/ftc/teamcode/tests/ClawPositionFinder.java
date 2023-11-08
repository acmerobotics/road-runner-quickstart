package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.settings.GamepadSettings;

@TeleOp(name="ClawPositionFinder", group="Tests")
public class ClawPositionFinder extends OpMode {

    Claw claw;

    @Override
    public void init() {
        claw = new Claw();
        claw.init(hardwareMap);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        if (gamepad1.left_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE) {
            claw.leftProng.setPosition(claw.leftProng.getPosition() + 0.01);
        } else if (gamepad1.right_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE) {
            claw.rightProng.setPosition(claw.rightProng.getPosition() + 0.01);
        } else if (gamepad1.left_bumper) {
            claw.leftProng.setPosition(claw.leftProng.getPosition() - 0.01);
        } else if (gamepad1.right_bumper) {
            claw.rightProng.setPosition(claw.rightProng.getPosition() - 0.01);
        }

        if (gamepad1.a) {
            claw.rotator.setPosition(claw.rotator.getPosition() + 0.01);
        } else if (gamepad1.b) {
            claw.rotator.setPosition(claw.rotator.getPosition() - 0.01);
        }

//        claw.loop(gamepad1);
        claw.telemetry(telemetry);
        telemetry.update();
    }
}