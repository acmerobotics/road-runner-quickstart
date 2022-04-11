package org.firstinspires.ftc.teamcode.opMode.protoType;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.SCORINGFSM;

@TeleOp
public class ZawadBreaksScoring extends LinearOpMode {
    SCORINGFSM oppenheimer = new SCORINGFSM();
    @Override
    public void runOpMode() throws InterruptedException {
        oppenheimer.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.a) {
                oppenheimer.highGoal();
            }
            if (gamepad1.b) {
                oppenheimer.lowGoal();
            }
            if (gamepad1.x) {
                oppenheimer.midGoal();
            }
            if (gamepad1.y) {
                oppenheimer.score();
            }
            if (gamepad1.left_stick_button) {
                oppenheimer.down();
            }
            oppenheimer.loop();
            telemetry.addData("bruh", oppenheimer.scoreStates);
            telemetry.addData("arm", oppenheimer.arm.armStates);
            telemetry.addData("lift", oppenheimer.lift.liftState);
            telemetry.update();
        }
    }
}
