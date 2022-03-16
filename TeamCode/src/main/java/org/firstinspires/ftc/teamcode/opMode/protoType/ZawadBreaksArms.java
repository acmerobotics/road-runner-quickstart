package org.firstinspires.ftc.teamcode.opMode.protoType;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.ARMFSM;

@TeleOp
public class ZawadBreaksArms extends LinearOpMode {
    ARMFSM worldbreaker = new ARMFSM();
    @Override
    public void runOpMode() throws InterruptedException {
        worldbreaker.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.a) {
                worldbreaker.down();
            }
            if(gamepad1.b) {
                worldbreaker.ready();
            }
            if(gamepad1.x) {
                worldbreaker.dump();
            }
            worldbreaker.loop();
            telemetry.addData("state", worldbreaker.armStates);
            telemetry.update();
        }
    }
}
