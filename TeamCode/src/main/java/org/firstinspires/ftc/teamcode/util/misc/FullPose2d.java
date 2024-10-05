package org.firstinspires.ftc.teamcode.util.misc;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

public class FullPose2d {
    double x;
    double y;
    double heading;
    double intakeExtension;

    public FullPose2d(double x1, double y1, double heading1) {
        x = x1;
        y = y1;
        heading = heading1;
    }

    public FullPose2d(double x1, double y1, double heading1, double intakeExt) {
        x = x1;
        y = y1;
        heading = heading1;
        intakeExtension = intakeExt;
    }

    public Pose2d getRobotPose() {
        return new Pose2d(x, y, new Rotation2d(heading));
    }
}
