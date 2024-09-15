package org.firstinspires.ftc.teamcode.unknownFiles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * Test the dashboard gamepad integration.
 */
@Autonomous
public class GamepadTestOpMode extends LinearOpMode {

    private static void logGamepad(Telemetry telemetry, Gamepad gamepad, String prefix) {
        telemetry.addData(prefix + "Synthetic",
            gamepad.getGamepadId() == Gamepad.ID_UNASSOCIATED);
        for (Field field : gamepad.getClass().getFields()) {
            if (Modifier.isStatic(field.getModifiers())) {
                continue;
            }

            try {
                telemetry.addData(prefix + field.getName(), field.get(gamepad));
            } catch (IllegalAccessException e) {
                // ignore for now
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            logGamepad(telemetry, gamepad1, "gamepad1");
            logGamepad(telemetry, gamepad2, "gamepad2");
            telemetry.update();

            sleep(20);
        }
    }
}
