package org.firstinspires.ftc.teamcode.Purepursuit.AStar;

import android.os.Build;

import androidx.annotation.RequiresApi;

import org.firstinspires.ftc.teamcode.Purepursuit.CurvePoint;

import java.util.ArrayList;

public class Main {


	@RequiresApi(api = Build.VERSION_CODES.N)
	public static void main(String[] args) {

		AStar astar = new AStar(new CurvePoint(36,48), new CurvePoint(0,0));
		astar.addCircleObstacle(-12,36,20);
		astar.addCircleObstacle(12,36,20);

		ArrayList<CurvePoint> path = astar.computeAStar();
		if (path.isEmpty()) return;
		astar.makePathStable(path);
		for (CurvePoint n: path) {
			System.out.println(n.moveSpeed);
		}
		System.out.println("path length" + path.size());
		System.out.println("last coordinate " + path.get(path.size() -1));
	}

}
