package org.firstinspires.ftc.teamcode.Testing;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp
public class TestColorSensor extends LinearOpMode {
    public static double yellowRedb = 0.2;
    public static double yellowGreenb = 0.28;
    public static double yellowBluet = 0.42;
    public static double redRedb = 0.15;
    public static double redGreent = 0.4;
    public static double redBluet = 0.3;
    public static double blueRedt = 0.3;
    public static double blueGreent = 0.4;
    public static double blueBlueb = 0.15;
    public static float gain = 50;
    @Override
    public void runOpMode() {

        NormalizedColorSensor colorSensor;


        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry tele = dashboard.getTelemetry();


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

            if (colors.red > redRedb && colors.green < redGreent && colors.blue < redBluet) {
                telemetry.addData("color","red");
                tele.addData("color","red");
            } else if (colors.red > yellowRedb && colors.green > yellowGreenb && colors.blue < yellowBluet) {
                telemetry.addData("color","yellow");
                tele.addData("color","yellow");
            } else if (colors.red < blueRedt && colors.green < blueGreent && colors.blue > blueBlueb) {
                telemetry.addData("color","blue");
                tele.addData("color","blue");
            } else {
                telemetry.addData("color","none");
                tele.addData("color","none");
            }

            //timer.reset();
            telemetry.addData("redv", colors.red);
            telemetry.addData("bluev", colors.blue);
            telemetry.addData("greenv", colors.green);
            tele.addData("redv", colors.red);
            tele.addData("bluev", colors.blue);
            tele.addData("greenv", colors.green);
            tele.update();
            telemetry.update();

        }
    }

}