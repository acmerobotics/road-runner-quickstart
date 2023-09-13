/*
 * copyright 2023 FTC21180
 * */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto Blue Back", group="Concept")
//@Disabled
public class AutoBlueBack extends AutoRedFront {
    @Override
    public void setRobotLocation() {
        startLoc = 4;
    }
}
