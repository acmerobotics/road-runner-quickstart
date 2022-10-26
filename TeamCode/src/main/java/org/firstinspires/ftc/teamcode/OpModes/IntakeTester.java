package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class IntakeTester extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		CRServo servo = hardwareMap.get(CRServo.class, "intake");

		ElapsedTime timer = new ElapsedTime();
		waitForStart();
		while (opModeIsActive()) {
			if (timer.seconds() < 3) {
				servo.setPower(1);
			} else if (timer.seconds() < 6) {
				servo.setPower(-1);
			} else {
				timer.reset();
			}

		}
	}
}
