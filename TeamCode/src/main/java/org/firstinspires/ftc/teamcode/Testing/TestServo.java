package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.Claw;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class TestServo extends LinearOpMode {

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void runOpMode() {

//        Servo left = hardwareMap.get(Servo.class, "intakeServoLeft");
//        Servo right = hardwareMap.get(Servo.class, "intakeServoRight");
          //Servo bucket = hardwareMap.get(Servo.class, "bucketServo");
          //Servo servoExtendo = hardwareMap.get(Servo.class, "servoExtendo");
        Claw claw = new Claw(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            if (gamepad1.a) {
//                left.setPosition(0.83);
//                right.setPosition(0.83);
                runningActions.add(claw.open());
            }

            if (gamepad1.x) {
//                left.setPosition(0.5);
//                right.setPosition(0.5);
                runningActions.add(claw.close());
            }

            if (gamepad1.b) {
//                left.setPosition(0.05);
//                right.setPosition(0.05);b

            }

            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;
            dash.sendTelemetryPacket(packet);
        }
    }

}
