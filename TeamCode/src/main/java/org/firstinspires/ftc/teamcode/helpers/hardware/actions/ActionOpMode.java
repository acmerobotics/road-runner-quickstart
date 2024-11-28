package org.firstinspires.ftc.teamcode.helpers.hardware.actions;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.List;

public abstract class ActionOpMode extends LinearOpMode {
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    protected void runBlocking(Action a) {
        Canvas c = new Canvas();
        a.preview(c);

        boolean b = true;
        while (b && !isStopRequested()) {
            TelemetryPacket p = new TelemetryPacket();
            p.fieldOverlay().getOperations().addAll(c.getOperations());

            b = a.run(p);

            dash.sendTelemetryPacket(p);
        }
    }

    protected void updateAsync(TelemetryPacket packet) {

        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        System.out.println(runningActions.toString() + newActions.toString() + "12087");
        runningActions = newActions;
    }

    protected void run(Action a) {
        runningActions.add(a);
    }


}