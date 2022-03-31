package org.firstinspires.ftc.teamcode.opMode.protoType;

import android.graphics.Paint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.hardware.CapVision;
@TeleOp (group = "prototype")
public class capTest extends LinearOpMode {
    CapVision vision = new CapVision();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {

        vision.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            //dashboard.startCameraStream(vision, 0);
            telemetry.addData("region?", vision.whichRegion());
            telemetry.update();
        }
    }
}
