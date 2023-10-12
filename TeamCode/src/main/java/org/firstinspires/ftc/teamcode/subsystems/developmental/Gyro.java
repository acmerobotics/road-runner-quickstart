package org.firstinspires.ftc.teamcode.subsystems.developmental;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Mechanism;

public class Gyro extends Mechanism {

    NavxMicroNavigationSensor gyro;

    public String gyroName = "gyro";

    @Override
    public void init(HardwareMap hwMap) {
        hwMap.get(NavxMicroNavigationSensor.class, gyroName);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Gyro Calibration Status: ", gyro.isCalibrating());
    }

    public String formatRate(float rate) {
        return String.format("%.3f", rate);
    }

    public String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public String formatDegrees(double degrees){
        return String.format("%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
