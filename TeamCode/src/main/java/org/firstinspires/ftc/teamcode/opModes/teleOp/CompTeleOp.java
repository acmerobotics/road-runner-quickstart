package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name="CompTeleOp", group="Tests")
public class CompTeleOp extends OpMode {

    Robot robot;

    @Override
    public void init() {
        robot = new Robot(true);
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        robot.loop(gamepad1, gamepad2);
        robot.telemetry(telemetry);
        telemetry.update();
    }
}