package org.firstinspires.ftc.teamcode.drive.opmode.ManualOp.manuel;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class resetEncoders extends LinearOpMode {
    private org.firstinspires.ftc.teamcode.drive.opmode.helpers.Slide slides;

    @Override
    public void runOpMode() {
        slides = new org.firstinspires.ftc.teamcode.drive.opmode.helpers.Slide(hardwareMap, telemetry);
        slides.initArm();
    }
}
