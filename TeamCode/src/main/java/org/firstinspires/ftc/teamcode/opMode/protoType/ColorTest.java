package org.firstinspires.ftc.teamcode.opMode.protoType;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.FreightSensor;

@TeleOp (group = "prototype")
public class ColorTest extends LinearOpMode {
    FreightSensor color = new FreightSensor();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dash = dashboard.getTelemetry();
    @Override
    public void runOpMode() {
        boolean hadFreight = false;
        color.init(hardwareMap);
        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) { ;

            telemetry.addData("blocc", color.hasFreight());
            dash.addData("blue", color.blue());
            dash.addData("red", color.red());
            dash.addData("green", color.green());
            dash.addData("block?", color.hasBlock());
            telemetry.update();
            dash.update();

            if(color.hasFreight()) {
                hadFreight = true;
                gamepad1.rumble(50, 50, 200);
            }

            if(hadFreight) {
                if(!color.hasFreight()) {
                    gamepad1.rumble(0, 0, 5);
                    hadFreight = false;
                }
            }
        }
    }
}
