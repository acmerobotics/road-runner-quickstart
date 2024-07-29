package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Opmodes.Testing$Tuning.Subsystems$Tele.CameraAutoMove;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="CameraData", group="Linear OpMode")
public class CameraData extends LinearOpMode {

    //private final robot robot = robot.getHardware();

    @Override
    public void runOpMode() throws InterruptedException {

/*
        robot.init(hardwareMap);
        Camera.init();

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();


        waitForStart();
        while (opModeIsActive()) {
            Camera.visionPortal.resumeLiveView();


            //untouched camera data
            double[] Raw = Camera.DetectAprilTagRawData();
            telemetry.addData("Raw Data - Yaw: ", Raw[0]);
            telemetry.addData("Raw Data - Range: ", Raw[1]);
            telemetry.addData("Raw Data - Bearing: ", Raw[2]);
            telemetry.addData("Xchange - Relative to Camera: ", Camera.Xchange);
            telemetry.addData("Ychange- Relative to Camera: ", Camera.Ychange);


            // Apply offsets
            double[] Translated = Camera.TranslateCamData(Camera.DetectAprilTagRawData());
            telemetry.addData("Updated Range: ", Translated[3]);
            telemetry.addData("Xchange - Relative to Updated Range: ", Translated[0]);
            telemetry.addData("Ychange - Relative to Updated Range: ", Translated[1]);
            telemetry.addData("Turn", Translated[2]);

            telemetry.update();
        }

    }

 */


    }
}