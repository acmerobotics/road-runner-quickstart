package org.firstinspires.ftc.teamcode.TestArchive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class TestTelemetry extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry tele = dashboard.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("a", "a");
            telemetry.update();

            }


        }
    }









