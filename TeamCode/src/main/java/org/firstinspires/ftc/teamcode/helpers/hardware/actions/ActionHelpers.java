package org.firstinspires.ftc.teamcode.helpers.hardware.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;

import java.util.function.BooleanSupplier;

public class ActionHelpers {
    public static class RaceParallelCommand implements Action {
        private final Action[] actions;

        public RaceParallelCommand(Action... actions) {
            this.actions = actions;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket t) {
            boolean finished = true;
            for (Action action : actions) finished = finished && action.run(t);
            return finished;
        }

        @Override
        public void preview(@NonNull Canvas canvas) {
            for (Action action : actions) action.preview(canvas);
        }


    }

    public static class TimeoutAction implements Action {
        private final Action action;
        private final double timeout;
        private double startTime;

        public TimeoutAction(@NonNull Action action, long timeout) {
            this.action = action;
            this.timeout = timeout;
        }

        public TimeoutAction(@NonNull Action action) {
            this.action = action;
            this.timeout = 5;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket t) {
            if (startTime == 0) startTime = Actions.now();
            if (Actions.now() - startTime > timeout) return false;
            return action.run(t);
        }

        @Override
        public void preview(@NonNull Canvas canvas) {
            action.preview(canvas);
        }
    }

    public static class WaitUntilAction implements Action {
        private final BooleanSupplier condition;

        public WaitUntilAction(BooleanSupplier condition) {
            this.condition = condition;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // Return true when the condition is met (action is complete)
            return condition.getAsBoolean();
        }
    }

}