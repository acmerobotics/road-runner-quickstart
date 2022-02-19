package org.firstinspires.ftc.teamcode.opMode.protoType;

import android.graphics.Paint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.hardware.CapVision;
@TeleOp
public class capTest extends LinearOpMode {
    CapVision vision = new CapVision();

    @Override
    public void runOpMode() throws InterruptedException {
        vision.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("region?", vision.whichRegion());
            telemetry.update();
        }
    }
}
