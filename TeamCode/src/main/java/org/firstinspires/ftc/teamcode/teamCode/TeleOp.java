package org.firstinspires.ftc.teamcode.teamCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Config
public class TeleOp extends LinearOpMode {

    StickyGamepad sticky1;
    StickyGamepad sticky2;
    ChassisController sasiu;
    ArmController arm;

    LiftController lift;

    ClawController claw;

    @Override
    public void runOpMode() throws InterruptedException {

        sasiu = new ChassisController(hardwareMap);
        arm = new ArmController(hardwareMap);
        lift = new LiftController(hardwareMap);
        claw = new ClawController(hardwareMap);
        sticky1 = new StickyGamepad(gamepad1, this);
        sticky2 = new StickyGamepad(gamepad2, this);

        waitForStart();

        while(opModeIsActive())
        {
            sasiu.move(gamepad1);
            arm.update();
            lift.update();
            sticky1.update();
            sticky2.update();

            if(gamepad1.a)
            {
                arm.goDown();
            }
            if(gamepad1.b)
            {
                arm.goMid();
            }
            if(gamepad1.y)
            {
                arm.goUp();
            }
            if (gamepad1.left_trigger != 0) {
                lift.down(gamepad1.left_trigger);
            }
            if (gamepad1.right_trigger != 0) {
                lift.up(gamepad1.right_trigger); }
            if (gamepad1.dpad_down) {
                lift.goDown(); }
            if (gamepad1.dpad_up) {
                lift.goUp(); }
            if (gamepad1.dpad_left) {
                lift.goMid(); }
            if (sticky1.right_bumper) {
                claw.toggleRight();
            }
            if (sticky1.left_bumper) {
                claw.toggleLeft();
            }
        }
    }
}
