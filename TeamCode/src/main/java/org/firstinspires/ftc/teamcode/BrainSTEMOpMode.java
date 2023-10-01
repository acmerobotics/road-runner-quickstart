package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;  // TODO: fix this
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;


abstract public class BrainSTEMOpMode extends CommandOpMode {
    protected MecanumDrive drivetrain;
    protected NormalizedColorSensor colorSensor;
    protected DistanceSensor distanceSensor;

    protected GamepadEx driverOne;
    protected GamepadEx driverTwo;

    protected void initHardware() {
        drivetrain = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        colorSensor     = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        distanceSensor  = hardwareMap.get(DistanceSensor.class, "sensor_distance");

        driverOne = new GamepadEx(gamepad1);
        driverTwo = new GamepadEx(gamepad2);
    }
}
