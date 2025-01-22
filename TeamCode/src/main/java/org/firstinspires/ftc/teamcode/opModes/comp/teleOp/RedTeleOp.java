package org.firstinspires.ftc.teamcode.opModes.comp.teleOp;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opModes.comp.auto.supers.SupersAutoConstants;
import org.firstinspires.ftc.teamcode.subsystems.v1.Robot;

@TeleOp(name="RedTeleOp", group="AAA_COMP")
public class RedTeleOp extends OpMode {

    Robot robot;
    AIMPad aimPad1;
    AIMPad aimPad2;

    @Override
    public void init() {
        robot = new Robot(true, SupersAutoConstants.BLUE_STARTING_POSITION);
        robot.init(hardwareMap);
        aimPad1 = new AIMPad(gamepad1);
        aimPad2 = new AIMPad(gamepad2);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        robot.loop(aimPad1, aimPad2);
        aimPad1.update(gamepad1);
        aimPad2.update(gamepad2);

        telemetry.addData("Previous State", aimPad1.getPreviousState());
        telemetry.addData("Current State", aimPad1.getCurrentState());
        robot.telemetry(telemetry);
        telemetry.update();
    }
}