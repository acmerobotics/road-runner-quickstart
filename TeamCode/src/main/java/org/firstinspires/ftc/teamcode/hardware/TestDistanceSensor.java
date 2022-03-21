package org.firstinspires.ftc.teamcode.hardware;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.math.*;
import org.checkerframework.checker.units.qual.K;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class TestDistanceSensor extends Mechanism {
    DistanceSensor distanceSensor;

    public static double robotLength = 0;

    //offset in inches
    public static double sensorOffset = 0;
    public TestDistanceSensor(LinearOpMode opMode) { this.opMode = opMode; }

    // higher gain values -> smoother graph (more averaged values) but
    // significantly higher lag
    public static double GAIN = 0.2;
    LowPassFilter lowPassFilter = new LowPassFilter(GAIN);

    // Kalman filter
    public static double Q = 0.3;
    public static double R = 3;
    public static int N = 3;
    KalmanFilter kalmanFilter = new KalmanFilter(Q,R,N);

    @Override
    public void init(HardwareMap hwMap) {
        distanceSensor = hwMap.get(DistanceSensor.class, "distance");
    }

    public double getCM() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }

    public double getIN(){
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }

    public double getLowPass() {
        double currentValue = distanceSensor.getDistance(DistanceUnit.CM);
        double estimate = lowPassFilter.estimate(currentValue);
        return estimate;
    }

    public double getKalman(DistanceUnit unit) {
        double offset = sensorOffset;
        if(unit.equals(DistanceUnit.CM)) offset *= 2.54;

        double currentValue = distanceSensor.getDistance(unit);  // imaginary, noisy sensor
        double estimate = kalmanFilter.estimate(currentValue) + offset; // smoothed sensor
        return estimate;
    }

    public double getKalman(){
        return getKalman(DistanceUnit.INCH);
    }

    public double localizationRetune(double heading){

        return 0.0;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("CM", getCM());
        telemetry.addData("Low Pass", getLowPass());
        telemetry.addData("Kalman", getKalman());
        telemetry.addData("Low Pass Offset", Math.abs(getCM() - getLowPass()));
        telemetry.addData("Kalman Offset", Math.abs(getCM() - getKalman()));
    }
}