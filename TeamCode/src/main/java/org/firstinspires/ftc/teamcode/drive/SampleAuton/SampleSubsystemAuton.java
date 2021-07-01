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
        drive.shooter(-1);
        sleep(1000);
        for(int i=0;i<3;++i){
           drive.slapper.setPosition(0.35);
            sleep(500);
            drive.slapper.setPosition(0.65);
            sleep(500);
        }
        drive.shooter(0);
    }

}
