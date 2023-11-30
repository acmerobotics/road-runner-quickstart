package org.firstinspires.ftc.teamcode.testing.structureOptions.inheritanceStructure;

import com.acmerobotics.dashboard.FtcDashboard;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Dashboard test", group="Linear Opmode")
public class FTCdashboard extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode() {

        telemetry.addLine("stuff");
        telemetry.update();

        waitForStart();
    }
}
