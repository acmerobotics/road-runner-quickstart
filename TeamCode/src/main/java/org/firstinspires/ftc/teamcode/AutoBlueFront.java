/*
* copyright 2023 FTC21180
* */
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto Blue Front", group="Concept")
//@Disabled
public class AutoBlueFront extends AutoRedFront {
    @Override
    public void setRobotLocation() {
        startLoc = 3;
    }
}
