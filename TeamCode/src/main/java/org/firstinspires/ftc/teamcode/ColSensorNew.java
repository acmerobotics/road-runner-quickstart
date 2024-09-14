package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp
public class ColSensorNew extends LinearOpMode {

    ColorSensor color;
    int redvalue; //raw color values
    int bluevalue;
    int greenvalue;
    double distance; // largest when an object is close to the sensor, range of 0-2047
    String currentColor;


    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        color = hardwareMap.get(ColorSensor.class, "Color");

        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) {
            redvalue = color.red();
            greenvalue = color.green();
            bluevalue = color.blue();
            distance = ((DistanceSensor) color).getDistance(DistanceUnit.CM);



            telemetry.addData("Red: ", redvalue);
            telemetry.addData("Green: ", greenvalue);
            telemetry.addData("Blue: ", bluevalue);
            telemetry.addData("Distance: ", distance);
            telemetry.update();
        }
    }
}
