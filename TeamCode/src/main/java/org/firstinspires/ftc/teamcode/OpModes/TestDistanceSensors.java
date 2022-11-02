package org.firstinspires.ftc.teamcode.OpModes;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Autonomous
public class TestDistanceSensors extends LinearOpMode {
	KalmanFilter filter = new KalmanFilter(0.3,0.5,3);
	@Override
	public void runOpMode() throws InterruptedException {
		AnalogInput distance1 = hardwareMap.get(AnalogInput.class, "Distance1");
		AnalogInput distance2 = hardwareMap.get(AnalogInput.class, "Distance2");
//		double estimate = filter.estimate(distance1.getVoltage());
		waitForStart();
		while(opModeIsActive()) {
			System.out.println("V1: " + distance1.getVoltage() + " V2: " + distance2.getVoltage());
		}
	}
}
