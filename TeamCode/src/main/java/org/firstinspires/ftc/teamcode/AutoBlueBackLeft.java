/*
 * copyright 2023 FTC21180
 * */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Back Left", group="Concept")
//@Disabled
public class AutoBlueBackLeft extends AutoRedFrontLeft {
    @Override
    public void setRobotLocation() {
        startLoc = 4;
        leftOrRight = -1;
    }
}
