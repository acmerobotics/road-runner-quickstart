package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Stopwatch {
    ElapsedTime time;
    double offset = 0;
    boolean paused = false;
    double pauseStartTime = 0;

    public Stopwatch() {
        time = new ElapsedTime();
    }

    public double seconds() {
        if (paused) return pauseStartTime + offset;

        return time.seconds() + offset;
    }

    public void setPaused() {
        if (paused) return;

        paused = true;
        pauseStartTime = time.seconds();
    }

    public void setUnpaused() {
        if (!paused) return;

        paused = false;
        offset -= time.seconds() - pauseStartTime;
    }
}
