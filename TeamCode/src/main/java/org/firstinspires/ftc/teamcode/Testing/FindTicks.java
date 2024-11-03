package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.Extendo;
import org.firstinspires.ftc.teamcode.mechanisms.Intaker;
import org.firstinspires.ftc.teamcode.mechanisms.Slides;

@TeleOp
public class FindTicks extends LinearOpMode {

    //Slides slides = new Slides(hardwareMap);

    @Override
    public void runOpMode() {

        Extendo extendo = new Extendo(hardwareMap);
        Slides slides = new Slides(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("extendo", extendo.getPos());
            telemetry.addData("slides", slides.getPos());
            telemetry.update();
        }
    }
}
