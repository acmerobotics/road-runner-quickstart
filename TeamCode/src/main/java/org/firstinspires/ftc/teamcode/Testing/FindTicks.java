package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.Extendo;
import org.firstinspires.ftc.teamcode.mechanisms.Intaker;
import org.firstinspires.ftc.teamcode.mechanisms.Slides;

@TeleOp
public class FindTicks extends LinearOpMode {

    //Slides slides = new Slides(hardwareMap);

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry tele = dashboard.getTelemetry();


        Extendo extendo = new Extendo(hardwareMap);
        Slides slides = new Slides(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            tele.addData("extendo", extendo.getPos());
            tele.addData("slides", slides.getPos());
            tele.update();
            telemetry.addData("extendo", extendo.getPos());
            telemetry.addData("slides", slides.getPos());
            telemetry.update();
        }
    }
}
