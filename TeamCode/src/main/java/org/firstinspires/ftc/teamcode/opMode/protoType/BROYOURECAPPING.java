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
                cap.reset();
            }
            if(gamepad1.b) {
                cap.grabCap();
            }
            if(gamepad1.x) {
                cap.raise();
            }
<<<<<<< HEAD
//            telemetry.addData("endpos", cap.capping);
//            telemetry.addData("idlepos", cap.idle);
//            telemetry.addData("grabpos", cap.grab);
=======
            telemetry.addData("endpos", cap.CAPPING_DOWN);
            telemetry.addData("idlepos", cap.IDLE);
            telemetry.addData("grabpos", cap.CAPPING_UP);
>>>>>>> 3b930a15133b75bb36d6896e6ab92356fa0e9447
        }
    }
}
