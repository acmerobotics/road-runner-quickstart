package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class linearOpBenchmark extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		waitForStart();
		ElapsedTime timer = new ElapsedTime();
		while (opModeIsActive()) {
			System.out.println("DT: " + timer.seconds());
			timer.reset();
		}
	}
}
