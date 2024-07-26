package org.firstinspires.ftc.teamcode.Outdated_CenterStage.Our.Teleop.NeedToTest;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BlinkingLearning")

@Disabled
public class ZBlinkingLearning extends LinearOpMode {

    @Override
    public void runOpMode() {
        RevBlinkinLedDriver lights;
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");


        while (opModeIsActive()) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        }


    }


}
