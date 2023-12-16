/*
 * copyright 2023 FTC21180
 * */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Back Right", group="Concept")
//@Disabled
public class AutoBlueBackRight extends AutoRedFrontLeft {
    @Override
    public void setRobotLocation() {
        startLoc = 4;
        leftOrRight = 1;
    }
}
