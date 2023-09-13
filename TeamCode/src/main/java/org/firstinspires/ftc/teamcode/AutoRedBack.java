/*
 * copyright 2023 FTC21180
 * */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto Red Back", group="Concept")
//@Disabled
public class AutoRedBack extends AutoRedFront {
    @Override
    public void setRobotLocation() {
        startLoc = 2;
    }
}
