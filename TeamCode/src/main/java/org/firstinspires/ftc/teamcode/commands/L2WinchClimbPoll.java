package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.enums.Levels;

public class L2WinchClimbPoll implements Action {
    Robot robot;
    Gamepad gamepad;

    public L2WinchClimbPoll(Robot r, Gamepad gamepad) {
        robot = r;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (robot.imu.getRobotYawPitchRollAngles().getRoll() > 30 || robot.l3ClimbOverride) {
            robot.state = Levels.CLIMB_L2;
            gamepad.rumble(200);
            return false;
        }
        return true;
    }
}
