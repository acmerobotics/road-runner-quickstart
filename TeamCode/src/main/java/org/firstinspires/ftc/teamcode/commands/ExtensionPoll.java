package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.enums.Levels;

public class ExtensionPoll implements Action {
    Robot robot;
    float ticks;

    public ExtensionPoll(Robot r, float ticks) {
        robot = r;
        this.ticks = ticks;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (Math.abs(robot.extension.getPosition() - ticks) < 2) {
            return false;
        }
        return true;
    }
}
