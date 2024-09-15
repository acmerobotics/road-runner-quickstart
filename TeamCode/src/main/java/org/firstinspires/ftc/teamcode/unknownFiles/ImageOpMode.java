package org.firstinspires.ftc.teamcode.unknownFiles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class ImageOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        //FtcDashboard.setDrawDefaultField(false);
        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().drawImage("/images/ftc.jpg", 0, 0, 144, 144);
            //packet.fieldOverlay().drawImage("/dash/powerplay.png", 0, 0, 144, 144);
            dashboard.sendTelemetryPacket(packet);
            sleep(20);
        }
    }
}
