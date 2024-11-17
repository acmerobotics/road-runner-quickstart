package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class LoopAction implements Action {
    Runnable function;

    /**
     * Action that just runs the <code>lambda</code> in a constant loop with no end
     * @param lambda function you want running in a loop
     */
    public LoopAction(Runnable lambda) {
        function = lambda;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
       function.run();
       return true;
    }
}
