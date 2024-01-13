package org.firstinspires.ftc.teamcode.testing.openCvVision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Test OpenCV", group="Linear Opmode")
public class Test extends LinearOpMode {

    public void runOpMode(){
        VisionHandler visionHandler = new VisionHandler();
        visionHandler.init(hardwareMap);

        waitForStart();

        try {
            visionHandler.setBlue();
            visionHandler.setClose();
            telemetry.addData("Blue Close: ", visionHandler.read());
            visionHandler.setBlue();
            visionHandler.setFar();
            telemetry.addData("Blue Far: ", visionHandler.read());
            visionHandler.setRed();
            visionHandler.setClose();
            telemetry.addData("Red Close: ", visionHandler.read());
            visionHandler.setRed();
            visionHandler.setFar();
            telemetry.addData("Red Far: ", visionHandler.read());
            telemetry.update();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        while(opModeIsActive()){

        }
    }
}
