package org.firstinspires.ftc.teamcode.teamCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Config
public class TeleOp extends LinearOpMode {

    boolean bratEJos = false;
    StickyGamepad sticky1;
    StickyGamepad sticky2;
    ChassisController sasiu;
    ArmController arm;
    JointController joint;
    LiftController lift;

    ClawController claw;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        sasiu = new ChassisController(hardwareMap);
        arm = new ArmController(hardwareMap);
        lift = new LiftController(hardwareMap);
        claw = new ClawController(hardwareMap);
        joint = new JointController(hardwareMap);
        joint.goToMid();
        arm.goMid();
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

            if(lift.currentPos > 400 && bratEJos)
            {
                arm.target = 110;
            }


            if(gamepad1.a)
            {
                bratEJos = true;
                arm.goDown();
                joint.goToDown();
                lift.goTOPos(300);
            }
            if(gamepad1.b)
            {
                bratEJos = false;
                arm.goMid();
                joint.goToMid();
                lift.goTOPos(0);
            }
            if(gamepad1.y)
            {
                bratEJos = false;
                arm.goUp();
                joint.goToUp();
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

            telemetry.addData("glis: ", lift.currentPos);
            telemetry.update();
        }
    }
}
