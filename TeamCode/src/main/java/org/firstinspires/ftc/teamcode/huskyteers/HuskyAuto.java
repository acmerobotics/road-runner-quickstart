package org.firstinspires.ftc.teamcode.huskyteers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Husky Auto", group = "Auto")
public class HuskyAuto extends HuskyBot {

    @Override
    public void runOpMode() {
        super.initializeHardware();

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {

        }
    }
}
