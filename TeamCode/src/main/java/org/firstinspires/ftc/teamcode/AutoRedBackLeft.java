/*
 * copyright 2023 FTC21180
 * */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Back Left", group="Concept")
//@Disabled
public class AutoRedBackLeft extends AutoRedFrontLeft {
    @Override
    public void setRobotLocation() {
        startLoc = 2;
        leftOrRight = -1;
    }
}
