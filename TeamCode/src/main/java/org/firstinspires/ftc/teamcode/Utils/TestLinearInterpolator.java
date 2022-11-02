package org.firstinspires.ftc.teamcode.Utils;



//detected Pole: Pole{xPixel=352.0, width=109.0, isValidPole=true}0,0

import java.util.ArrayList;

// detected Pole: Pole{xPixel=92.0, width=117.0, isValidPole=true} -0.1,2.5
// detected Pole: Pole{xPixel=562.0, width=113.0, isValidPole=true} -0.4,-1.8
// detected Pole: Pole{xPixel=555.0, width=129.0, isValidPole=true} 1.0793615056637846, -1.8024541968487422
// detected Pole: Pole{xPixel=434.0, width=73.0, isValidPole=true} -5.500307242023125,-1.0340386610282355
// detected Pole: Pole{xPixel=89.0, width=107.0, isValidPole=true}  -0.1701621247931212,  2.7879328083329105
//
public class TestLinearInterpolator {



	public static void main(String[] args) throws Exception {

		double [] xs = {0,2.5,-1.8,-1.8024541968487422,-1.0340386610282355,2.7879328083329105};
		ArrayList<Double> Positions = new ArrayList<>();
		for (double d: xs) {
			Positions.add(d);
		}

		double [] x2s = {352, 92.0,562.0, 555.0, 434.0, 89.0};
		ArrayList<Double> xPixels = new ArrayList<>();
		for (double d: x2s) {
			xPixels.add(d);
		}

		LinearInterpolator interpolator = new LinearInterpolator(xPixels,Positions);
		System.out.println("value is " + interpolator.getValue(94));






	}
}
