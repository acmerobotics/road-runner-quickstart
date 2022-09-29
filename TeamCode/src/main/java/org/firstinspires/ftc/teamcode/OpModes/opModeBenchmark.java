package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class opModeBenchmark extends OpMode {

	ElapsedTime timer = new ElapsedTime();
	@Override
	public void init() {
		timer.reset();
	}

	@Override
	public void loop() {
		System.out.println("DT: " + timer.seconds());
		timer.reset();
	}
}
