package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;

public class DistanceSensor extends Subsystem {

	KalmanFilter filter = new KalmanFilter(0.3,0.1,3);
	double constant = (1 / 0.003388888889);

	AnalogInput distance2;

	protected double distance_in = 0;

	@Override
	public void initAuto(HardwareMap hwMap) {
		distance2 = hwMap.get(AnalogInput.class, "Distance2");
	}

	@Override
	public void periodic() {
		double sensor_volts = distance2.getVoltage();
		double sensor_inches = sensor_volts * constant;
		distance_in = filter.estimate(sensor_inches);
		Dashboard.packet.put("DistanceKF",sensor_inches);
		Dashboard.packet.put("Sensor_Inches_raw",sensor_inches);
	}

	@Override
	public void shutdown() {

	}

	public double getDistance_in() {
		return distance_in;
	}
}
