package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.vision.CVMaster;

public class LocateTargetsCV implements Action {
    Robot robot;
    CVMaster.EOCVPipeline color;
    public LocateTargetsCV(Robot r, CVMaster.EOCVPipeline c) {
        robot = r;
        color = c;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (robot.lift.getPos() == 20) {
            robot.cv.updatePotentialTargetList(color, robot.drive.pose);
            return false;
        }
        return true;
    }
}
