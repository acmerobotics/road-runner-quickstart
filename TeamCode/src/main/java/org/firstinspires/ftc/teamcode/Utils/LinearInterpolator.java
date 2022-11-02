package org.firstinspires.ftc.teamcode.Utils;

import android.os.Build;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;

public class LinearInterpolator {

	ArrayList<Double> x;
	ArrayList<Double> y;
	HashMap<Double, Double> map;


	public LinearInterpolator(ArrayList<Double> x, ArrayList<Double> y) {
		this.x = x;
		this.y = y;



		map = new HashMap<>();
		for (int i = 0; i < x.size(); ++i) {
			map.put(x.get(i),y.get(i));
		}

		Collections.sort(this.x);
		System.out.println(this.x);

	}

	public double getValue(double key) {
		// nearest value

		// if key is less than the lowest value in x then return the first element of the hashmap
		if (x.get(0) >= key || x.size() < 2) {
			System.out.println("x at 0 is " + x.get(0));
			System.out.println("value is less than lowest value or " + (x.get(0) >= key));
			return safeObtain(x.get(0));
		}


		double currentLowerBound = x.get(0);
		double currentUpperBound = x.get(x.size() - 1);

		if (currentUpperBound < key) {
			System.out.println("is less than upper bound");
			return safeObtain(currentUpperBound);
		}

		for (int i = 1; i < x.size(); ++i) {
			if (x.get(i) < key){
				currentLowerBound = x.get(i);
			}
			if (x.get(i) > key) {
				currentUpperBound = x.get(i);
			}
		}


		System.out.println("is calculating using interpolation");
		double upperY = safeObtain(currentUpperBound);
		double lowerY = safeObtain(currentLowerBound);
		double xDifferenceTerm = (currentUpperBound - key) / (currentUpperBound - currentLowerBound);

		return lowerY * xDifferenceTerm + upperY * xDifferenceTerm;




	}

	protected double safeObtain(double key) {
		try {
			System.out.println("Key " + key + " accesses value: " + map.get(key));
			return map.get(key);
		} catch (NullPointerException e) {
			e.printStackTrace();
			return 0;
		}
	}


}
