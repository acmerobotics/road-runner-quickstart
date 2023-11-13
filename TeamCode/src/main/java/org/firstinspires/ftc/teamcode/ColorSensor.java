package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "color sensor test", group = "TeleOp")
public class ColorSensor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        NormalizedColorSensor colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color_1");
        waitForStart();

        while(opModeIsActive()){
            float gain = 10;

            colorSensor.setGain(gain);
            colorSensor1.setGain(gain);

            final float[] colorSensorValues = new float[3];
            final float[] colorSensorValue1 = new float[3];
            NormalizedRGBA colorSensorsColors  = colorSensor.getNormalizedColors();
            NormalizedRGBA colorSensor1Colors = colorSensor1.getNormalizedColors();

            Color.colorToHSV(colorSensorsColors.toColor(), colorSensorValues);
            Color.colorToHSV(colorSensor1Colors.toColor(), colorSensorValue1);

            double distanceColorSensor1 = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
            double distanceColorSensor2 = ((DistanceSensor) colorSensor1).getDistance(DistanceUnit.CM);
            telemetry.addData("Distance sensor 1(cm)", "%.3f", distanceColorSensor1);
            telemetry.addData("Distance sensor 2(cm)", "%.3f", distanceColorSensor2);
            if(distanceColorSensor1 == 0.636){
                telemetry.addLine("There is a pixel in the robot");
            }
            telemetry.update();
        }

    }

}

