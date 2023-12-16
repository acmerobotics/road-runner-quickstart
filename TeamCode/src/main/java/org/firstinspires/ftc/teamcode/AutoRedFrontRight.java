/*
 * copyright 2023 FTC21180
 * */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Front Right", group="Concept")
//@Disabled
public class AutoRedFrontRight extends AutoRedFrontLeft {
    @Override
    public void setRobotLocation() {
        startLoc = 1;
        leftOrRight = 1;
    }
}
