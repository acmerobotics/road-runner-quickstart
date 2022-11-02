package org.firstinspires.ftc.teamcode.visionPiplines;


public class Pole {
	public double xPixel;
	public double width;
	public boolean isValidPole;

	public Pole() {
		this.isValidPole = false;
		this.xPixel = 0;
		this.width = 0;
	}

	public Pole(double xPixel, double width) {
		this.xPixel = xPixel;
		this.width = width;
		this.isValidPole = true;
	}

	@Override
	public String toString() {
		return "Pole{" +
				"xPixel=" + xPixel +
				", width=" + width +
				", isValidPole=" + isValidPole +
				'}';
	}
}
