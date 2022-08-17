package org.firstinspires.ftc.teamcode.OpModes;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;

@Autonomous
public class MixedSquareTest extends BaseAuto {
	@RequiresApi(api = Build.VERSION_CODES.N)
	@Override
	public Command setupAuto(CommandScheduler scheduler) {
		double distance = 40;
		return drive(distance)
				.addNext(turn(Math.toRadians(90)))
				.addNext(drive(distance))
				.addNext(turn(Math.toRadians(180)))
				.addNext(drive(distance))
				.addNext(turn(Math.toRadians(270)))
				.addNext(drive(distance))
				.addNext(turn(0))

				.addNext(drive(distance))
				.addNext(turn(Math.toRadians(-90)))
				.addNext(drive(distance))
				.addNext(turn(Math.toRadians(-180)))
				.addNext(drive(distance))
				.addNext(turn(Math.toRadians(-270)))
				.addNext(drive(distance))
				.addNext(turn(-0));
	}
}
