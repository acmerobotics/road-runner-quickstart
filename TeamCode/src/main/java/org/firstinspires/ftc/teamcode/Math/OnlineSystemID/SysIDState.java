package org.firstinspires.ftc.teamcode.Math.OnlineSystemID;

/**
 * object to represent an identified state for system identification
 */
public class SysIDState {

	public final double power;
	public final double velocity;

	public SysIDState(double power, double velocity) {
		this.power = power;
		this.velocity = velocity;
	}
}
