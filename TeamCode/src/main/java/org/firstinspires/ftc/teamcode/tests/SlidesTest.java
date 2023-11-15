package org.firstinspires.ftc.teamcode.tests;


// This TeleOp tests the drivebase mechanism class

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.PIDSlides;
import org.firstinspires.ftc.teamcode.subsystems.settings.GamepadSettings;

@TeleOp(name="SlidesTest", group="Tests")
public class SlidesTest extends OpMode {

    PIDSlides slides;

    @Override
    public void init() {
        slides = new PIDSlides();
        slides.init(hardwareMap);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        if (Math.abs(gamepad1.left_stick_y) > GamepadSettings.GP2_STICK_DEADZONE) {
            slides.setPower(gamepad1.left_stick_y);
        } else if (gamepad1.a) {
            slides.update(-600);
        } else if (gamepad1.b) {
            slides.update(-1200);
        } else {
            slides.holdPosition();
        }
        slides.loop(gamepad1);
        slides.telemetry(telemetry);
        telemetry.update();
    }
}
