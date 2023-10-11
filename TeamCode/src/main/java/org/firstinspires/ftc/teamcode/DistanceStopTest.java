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

@TeleOp (name = "distance sensor stop test", group = "TeleOp")
public class DistanceStopTest extends LinearOpMode{
    DistanceSensor sensorDistance;
    @Override
    public void runOpMode() throws InterruptedException {

        float gain = 10;

        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        waitForStart();

        while(opModeIsActive()) {

            double driverWeight = 1.0;

            String distance = "stop";
            if (sensorDistance.getDistance(DistanceUnit.CM)>10 && sensorDistance.getDistance(DistanceUnit.CM)<20) { //adjust values
                distance = "near";
                if (distance.equalsIgnoreCase("near")) {
                    driverWeight = 0;
                }

                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y * driverWeight,
                                -gamepad1.left_stick_x * driverWeight
                        ),
                        -gamepad1.right_stick_x * driverWeight
                ));

                // generic DistanceSensor methods.
                telemetry.addData("deviceName", sensorDistance.getDeviceName());
                telemetry.addData("range", String.format("%.01f mm", sensorDistance.getDistance(DistanceUnit.MM)));
                telemetry.addData("range", String.format("%.01f cm", sensorDistance.getDistance(DistanceUnit.CM)));

                telemetry.update();

            }
        }
    }
}
