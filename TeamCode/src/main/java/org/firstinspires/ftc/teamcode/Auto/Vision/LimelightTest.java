package org.firstinspires.ftc.teamcode.Auto.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class LimelightTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Limelight limelight = new Limelight(hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("tx", limelight.update()[0]);
            telemetry.addData("ty", limelight.update()[1]);
            telemetry.update();
        }





    }


}
