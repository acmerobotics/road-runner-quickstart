package org.firstinspires.ftc.teamcode.Teleop;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class ColorSensorCode extends LinearOpMode {


    @Override
    public void runOpMode() {

        NormalizedColorSensor colorSensor;



        float gain = 50;

        final float[] hsvValues = new float[3];

        boolean xButtonPreviouslyPressed = false;
        boolean xButtonCurrentlyPressed = false;

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }



        // ElapsedTime timer = new ElapsedTime();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                gain += 0.005;
            } else if (gamepad1.b && gain > 1) {
                gain -= 0.005;
            }

            telemetry.addData("Gain", gain);
            colorSensor.setGain(gain);

            xButtonCurrentlyPressed = gamepad1.x;

            if (xButtonCurrentlyPressed != xButtonPreviouslyPressed) {
                if (xButtonCurrentlyPressed) {
                    if (colorSensor instanceof SwitchableLight) {
                        SwitchableLight light = (SwitchableLight)colorSensor;
                        light.enableLight(!light.isLightOn());
                    }
                }
            }
            xButtonPreviouslyPressed = xButtonCurrentlyPressed;

            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            Color.colorToHSV(colors.toColor(), hsvValues);

            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues[0])
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2]);
            telemetry.addData("Alpha", "%.3f", colors.alpha);

            if (colors.red > 0.5 && colors.green > 0.7 && colors.blue < 0.42) {
                telemetry.addData("Block Color", "yellow");
            } else if (colors.red > 0.5 && colors.green < 0.4 && colors.blue < 0.3) {
                telemetry.addData("Block Color", "red");
            } else if (colors.red < 0.3 && colors.green < 0.4 && colors.blue > 0.35) {
                telemetry.addData("Block Color", "blue");
            } else {
                telemetry.addData("Block Color", "none");
            }

            //timer.reset();
            telemetry.update();

        }
    }

}