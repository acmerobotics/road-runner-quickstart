package org.firstinspires.ftc.teamcode.drive.locolization;

import org.firstinspires.ftc.teamcode.util.math.Pose;

public interface SwerveLocalizer {

    void periodic();

    Pose getPos();

    void setPos(Pose pose);
}
