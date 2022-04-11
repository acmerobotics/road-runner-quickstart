package org.firstinspires.ftc.teamcode.opMode.protoType;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.LIFTFSM;

@TeleOp
public class ZawadBreaksLifts extends LinearOpMode {
    LIFTFSM destroyerOfWorlds = new LIFTFSM();
    Lift lift = new Lift();
    @Override
    public void runOpMode() throws InterruptedException {
        destroyerOfWorlds.init(hardwareMap, telemetry);
        lift.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.a) {
                destroyerOfWorlds.goHigh();
            }
            if(gamepad1.b) {
                destroyerOfWorlds.goLow();
            }
            if(gamepad1.x) {
                destroyerOfWorlds.goMid();
            }
            destroyerOfWorlds.loop();
            telemetry.addData("state", destroyerOfWorlds.liftState);
            telemetry.addData("bruh", destroyerOfWorlds.liftState == LIFTFSM.states.low);
            telemetry.addData("hmm", gamepad1.a);
            telemetry.update();
        }
    }
}
