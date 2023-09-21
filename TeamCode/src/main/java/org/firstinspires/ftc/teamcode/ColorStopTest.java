package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@TeleOp(name = "color sensor stop test", group = "TeleOp")
public class ColorStopTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        final float[] rgbvalues = new float[3];
        float gain = 10;

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        waitForStart();

        while (opModeIsActive()) {
            colorSensor.setGain(gain);

            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            double driverWeight = 1.0;

            String color = "none";
            if (colors.red>0.058 && colors.red<0.069){
                color = "red";
            }
            if (colors.blue>0.085 && colors.blue<0.1){
                color = "blue";
            }
            if (color.equalsIgnoreCase("blue") || color.equalsIgnoreCase("red")){
                driverWeight = 0;
            }

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y*driverWeight,
                            -gamepad1.left_stick_x*driverWeight
                    ),
                    -gamepad1.right_stick_x*driverWeight
            ));

            drive.updatePoseEstimate();

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", drive.pose.heading);
            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
            telemetry.update();


        }
    }
}