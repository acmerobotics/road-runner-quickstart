package org.firstinspires.ftc.teamcode.Outdated_CenterStage.Our.Teleop.NeedToTest;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "ColorSensor")

@Disabled
public class ZColorSensor extends LinearOpMode {

    // sensor and LED's
    private ZColorSensor daLight;
    // private LED red;
    // private LED green;
    RevBlinkinLedDriver lights;

    private void InitialSetup () {

        daLight = hardwareMap.get(ZColorSensor.class, "daLight");
        // red = hardwareMap.get(LED.class, "red");
        // green = hardwareMap.get(LED.class, "green");
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");

    }



    @Override
    public void runOpMode() {
        int gain;
        NormalizedRGBA normalizedColors;
        int color;
        float hue;
        float saturation;
        float value;
        gain = 2;


        InitialSetup();

        waitForStart();

        while (opModeIsActive()) {

            // Display distance info.
            telemetry.addData("Dist to tgt (mm)", ((DistanceSensor) daLight).getDistance(DistanceUnit.CM));
            // Display reflected light.

            if (gamepad1.a) {
                gain += 0.005;
            } else if (gamepad1.b && gain >= 1.005) {
                gain -= 0.005;
            }

            ((NormalizedColorSensor) daLight).setGain(gain);
            telemetry.addData("Gain", ((NormalizedColorSensor) daLight).getGain());

            // Gets RGB Color Values
            normalizedColors = ((NormalizedColorSensor) daLight).getNormalizedColors();
            telemetry.addData("Red", Double.parseDouble(JavaUtil.formatNumber(normalizedColors.red, 3)));
            telemetry.addData("Green", Double.parseDouble(JavaUtil.formatNumber(normalizedColors.green, 3)));
            telemetry.addData("Blue", Double.parseDouble(JavaUtil.formatNumber(normalizedColors.blue, 3)));

            // Convert RGB values to Hue, Saturation, and Value.
            color = normalizedColors.toColor();
            hue = JavaUtil.colorToHue(color);
            saturation = JavaUtil.colorToSaturation(color);
            value = JavaUtil.colorToValue(color);

            telemetry.addData("Hue", Double.parseDouble(JavaUtil.formatNumber(hue, 0)));
            telemetry.addData("Saturation", Double.parseDouble(JavaUtil.formatNumber(saturation, 3)));
            telemetry.addData("Value", Double.parseDouble(JavaUtil.formatNumber(value, 3)));
            telemetry.addData("Alpha", Double.parseDouble(JavaUtil.formatNumber(normalizedColors.alpha, 3)));
            // Show the color on the Robot Controller screen.
            JavaUtil.showColor(hardwareMap.appContext, color);

            //LINK https://www.youtube.com/watch?v=wMdkM2rr1a4
            //Hue LIGHTS
            //https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
            if (hue <= 350 && hue >= 200) {

                //red.enable(true);
               // green.enable(false);
                telemetry.addData("Pixel Color", "Purple");
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            }
            else if(hue <= 190 && hue >= 110) {
                //red.enable(false);
                //green.enable(true);
                telemetry.addData("Pixel Color", "Green");
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }
            else if(hue <= 100 && hue >= 45) {
                //red.enable(true);
                //green.enable(true);
                telemetry.addData("Pixel Color", "Yellow");
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            }
            else if(hue <= 30) {
                //red.enable(false);
                //green.enable(false);
                telemetry.addData("Pixel Color", "White");
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            }
            else {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }
            // Is it White CHECK
            if (saturation < 0.2) {
                //red.enable(false);
                //green.enable(false);
                telemetry.addData("Pixel Color", "White");
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            }


                /*
                Troubleshooting Telemetry

                 telemetry.addData("Light detected", ((OpticalDistanceSensor) daLight).getLightDetected());
                // Adjust the gain.

                 */

        }
        // Show white on the Robot Controller screen.
        JavaUtil.showColor(hardwareMap.appContext, Color.parseColor("white"));


    }
}