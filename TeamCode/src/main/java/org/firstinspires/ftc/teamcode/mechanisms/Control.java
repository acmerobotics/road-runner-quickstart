package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class Control {
    private boolean busy = false;
    private boolean finished = false;

    public Control() {

    }

    public void resetFinished() {
        finished = false;
    }

    public boolean getFinished() {
        return finished;
    }

    public boolean getBusy() {
        return busy;
    }

    public class Start implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            busy = true;
            return false;
        }
    }
    public Action start() {
        return new Start();
    }

    public class Done implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            busy = false;
            finished = true;
            return false;
        }
    }
    public Action done() {
        return new Done();
    }

}
