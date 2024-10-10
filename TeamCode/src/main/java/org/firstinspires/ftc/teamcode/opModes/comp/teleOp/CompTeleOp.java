package org.firstinspires.ftc.teamcode.opModes.comp.teleOp;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name="CompTeleOp", group="AAA_COMP")
public class CompTeleOp extends OpMode {

    Robot robot;
    AIMPad aimPad1;
    AIMPad aimPad2;
    Gamepad hi = new Gamepad();

    @Override
    public void init() {
        robot = new Robot();
        robot.init(hardwareMap);
        aimPad1 = new AIMPad(gamepad1);
        aimPad2 = new AIMPad(gamepad2);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        aimPad1.update(gamepad1);
        aimPad2.update(gamepad2);

        robot.loop(aimPad1, aimPad2);
        robot.telemetry(telemetry);
        telemetry.update();
    }
}