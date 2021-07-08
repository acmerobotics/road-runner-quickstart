package org.firstinspires.ftc.teamcode.drive.SampleAutonPaths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SampleSubsystemAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareFile drive = new HardwareFile(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;
        drive.leftIntakeHolder.setPosition(1);
        drive.rightIntakeHolder.setPosition(0);
        sleep(2000);
    }

}
