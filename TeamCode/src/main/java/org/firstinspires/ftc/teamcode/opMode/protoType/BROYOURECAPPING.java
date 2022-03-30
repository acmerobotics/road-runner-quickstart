package org.firstinspires.ftc.teamcode.opMode.protoType;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Capper;

public class BROYOURECAPPING extends LinearOpMode {
    Capper cap = new Capper();
    @Override
    public void runOpMode() throws InterruptedException {
        cap.init(hardwareMap);
        while(opModeIsActive()) {
            if(gamepad1.a) {
                cap.idle();
            }
            if(gamepad1.b) {
                cap.grabCap();
            }
            if(gamepad1.x) {
                cap.cap();
            }
            telemetry.addData("endpos", cap.capping);
            telemetry.addData("idlepos", cap.idle);
            telemetry.addData("grabpos", cap.grab);
        }
    }
}
