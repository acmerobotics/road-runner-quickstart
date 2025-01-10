package org.firstinspires.ftc.teamcode.TeleOp.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Robotv2;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Teleop with Actions", group="test")
@Config
public class TeleopWithActions extends OpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    Robotv2 robot = new Robotv2(hardwareMap, new Pose2d(0,0, Math.toRadians(0)));
    @Override
    public void init() {
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        // updated based on gamepads
        if (gamepad1.a) {
            runningActions.add(robot.scoreSampleAction());
        } else if (gamepad1.b) {
            runningActions.add(robot.comeDownAction());
        }

        // update running actions
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
