package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.settings.GamepadSettings;

@TeleOp(name="ArmPositionFinder", group="Tests")
public class ArmPositionFinder extends OpMode {

    Arm arm;

    @Override
    public void init() {
        arm = new Arm();
        arm.init(hardwareMap);
    }

    @Override
    public void start() {
        arm.leftArm.setPosition(arm.leftArm.getPosition());
        arm.rightArm.setPosition(arm.rightArm.getPosition());
    }

    @Override
    public void loop() {
        if (gamepad1.left_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE) {
            arm.leftArm.setPosition(arm.leftArm.getPosition() + 0.01);
            arm.rightArm.setPosition(arm.rightArm.getPosition() + 0.01);
        } else if (gamepad1.right_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE) {
            arm.leftArm.setPosition(arm.leftArm.getPosition() - 0.01);
            arm.rightArm.setPosition(arm.rightArm.getPosition() - 0.01);
        }

        arm.loop(gamepad1);
        arm.telemetry(telemetry);
        telemetry.update();
    }
}