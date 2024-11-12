package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class Presentation extends LinearOpMode {

    public class Claww {
        private Servo clawLeft;
        private Servo clawRight;

        public Claww(HardwareMap HWMap) {
            clawLeft = HWMap.get(Servo.class, "clawLeftServo");
            clawRight = HWMap.get(Servo.class, "clawRightServo");
        }

        public class Open implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawLeft.setPosition(0.3);
                clawRight.setPosition(0.8);
                return false;
            }
        }
        public Action open() {
            return new Open();
        }

        public class Close implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawLeft.setPosition(0.55);
                clawRight.setPosition(0.45);
                return false;
            }
        }
        public Action close() {
            return new Close();
        }
    }

    @Override
    public void runOpMode() {

        Claww claw = new Claww(hardwareMap);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        FtcDashboard dash = FtcDashboard.getInstance();
        List<Action> runningActions = new ArrayList<>();

        runningActions.add(claw.open());

        waitForStart();

        while (opModeIsActive()) {

            TelemetryPacket packet = new TelemetryPacket();
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (currentGamepad1.a && !previousGamepad1.a) {
                runningActions.add(new SequentialAction(
                        claw.close(),
                        claw.open(),
                        claw.close(),
                        claw.open()
                ));
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
