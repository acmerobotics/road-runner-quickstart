/*
* copyright 2023 FTC21180
* */
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Front Right", group="Concept")
//@Disabled
public class AutoBlueFrontRight extends AutoRedFrontLeft {
    @Override
    public void setRobotLocation() {
        startLoc = 3;
        leftOrRight = 1;
    }
}
