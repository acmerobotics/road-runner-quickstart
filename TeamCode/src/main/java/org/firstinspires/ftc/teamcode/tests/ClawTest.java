package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.developmental.PIDSlides;
import org.firstinspires.ftc.teamcode.subsystems.settings.GamepadSettings;

@TeleOp(name="ClawTest", group="Tests")
public class ClawTest extends OpMode {

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

//        claw.loop(gamepad1);
        claw.telemetry(telemetry);
        telemetry.update();
    }
}