package org.firstinspires.ftc.teamcode.MainCode.Autonomous.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="LCR TEST", group="Linear Opmode")
public class lcrTest extends LinearOpMode {

    public void runOpMode(){
        VisionHandler visionHandler = new VisionHandler();
        visionHandler.init(hardwareMap);

        waitForStart();

        try {
            visionHandler.setBlue();
            visionHandler.setLeft();
            telemetry.addData("Blue Left: ", visionHandler.read());
            visionHandler.setMiddle();
            telemetry.addData("Blue Middle: ", visionHandler.read());
            visionHandler.setRight();
            telemetry.addData("Blue Right: ", visionHandler.read());

            visionHandler.setRed();
            visionHandler.setLeft();
            telemetry.addData("Red Left: ", visionHandler.read());
            visionHandler.setMiddle();
            telemetry.addData("Red Middle: ", visionHandler.read());
            visionHandler.setRight();
            telemetry.addData("Red Right: ", visionHandler.read());

            telemetry.update();

        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        while(opModeIsActive()){

        }
    }
}
