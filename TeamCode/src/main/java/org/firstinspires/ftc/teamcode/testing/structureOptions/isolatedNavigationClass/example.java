
package org.firstinspires.ftc.teamcode.testing.structureOptions.isolatedNavigationClass;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Navigator example", group="Linear Opmode")

public class example extends LinearOpMode {

    Hardware hardware = new Hardware();
    Navigator nav = new Navigator();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();




    @Override
    public void runOpMode() {
        hardware.init(hardwareMap);
        nav.setHardware(hardware);
        hardware.getDT().setScls(0.6, 0.4);




        waitForStart();

        nav.forward(0.5, 10000);
    }
}
