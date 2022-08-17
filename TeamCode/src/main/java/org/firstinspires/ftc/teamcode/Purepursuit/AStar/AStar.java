package org.firstinspires.ftc.teamcode.Purepursuit.AStar;


import android.os.Build;

import androidx.annotation.RequiresApi;

import org.firstinspires.ftc.teamcode.Purepursuit.CurvePoint;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Objects;
import java.util.PriorityQueue;

import static java.util.Collections.max;
import static java.util.Collections.reverse;

@RequiresApi(api = Build.VERSION_CODES.N)
public class AStar {
	
	final int REASONABLE_INFINITY = 10000;

	final int FIELD_SIZE = 144;
	CurvePoint goal;
	CurvePoint initial;
	HashMap<CurvePoint, CurvePoint> cameFrom = new HashMap<>();
	HashMap<CurvePoint, Double> gScore = new HashMap<>();
	HashMap<CurvePoint, Double> fScore = new HashMap<>();
	PriorityQueue<CurvePoint> openSet = new PriorityQueue<>((point, t1) -> -Double.compare(getFScore(t1), getFScore(point)));

	public ArrayList<Circle> obstacles = new ArrayList<>();

	public CurvePoint[][] field;

	public AStar(CurvePoint goal, CurvePoint initial) {
		this.goal = goal;
		this.initial = initial;
		field = new CurvePoint[FIELD_SIZE + 1][FIELD_SIZE + 1];

		for (int x = -FIELD_SIZE/2; x <= FIELD_SIZE/2; ++x) {
			for (int y = -FIELD_SIZE/2; y <= FIELD_SIZE/2; ++y) {
				field[x+FIELD_SIZE/2][y+FIELD_SIZE/2] = new CurvePoint(x,y);
			}
		}
	}


	public ArrayList<CurvePoint> getNeighbors(CurvePoint point) {

		int [][] directions = {
				{1,0},
				{-1,0},
				{0,1},
				{0,-1},
				{1,1},
				{1,-1},
				{-1,-1},
				{-1,1}
		};

		int indexX = (int)point.x + (FIELD_SIZE / 2);
		int indexY = (int)point.y + (FIELD_SIZE / 2);
		ArrayList<CurvePoint> neighbors = new ArrayList<>();

		for (int[] direction : directions) {
			int newX = indexX + direction[0] ;
			int newY = indexY + direction[1] ;
			if (!(newX < 0 || newX > FIELD_SIZE  || newY < 0 || newY > FIELD_SIZE) ) {
				if (!field[newX][newY].isObstacle()) {
					neighbors.add(new CurvePoint(field[newX][newY]));
				}
			}
		}
		return neighbors;
	}





	/**
	 * straight line distance heuristic function
	 * @param n node we are calculating the weight of
	 * @return weight of n relative to the goal
	 */
	public double H(CurvePoint n) {
		//return n.distanceTo(goal);
		return Math.abs(n.x - goal.x) + Math.abs(n.y - goal.y);
	}

	public ArrayList<CurvePoint> computeAStar() {
		gScore.put(initial,0.0);
		fScore.put(initial, H(initial));
		openSet.add(initial);


		while (!openSet.isEmpty()) {

			CurvePoint current = openSet.peek();
			assert current != null;
			openSet.remove(current);

			if (current.equals(goal)) {
				return reconstructPath(cameFrom, current);
			}


			ArrayList<CurvePoint> neighbors = getNeighbors(current);

			System.out.println(neighbors.size());

			for (CurvePoint neighbor: neighbors) {
				double tentative_gScore = getGScore(current) + current.distanceTo(neighbor);
				if (tentative_gScore < getGScore(neighbor) && !neighbor.isObstacle()) {
					cameFrom.put(neighbor,current);
					gScore.put(neighbor, tentative_gScore);
					fScore.put(neighbor, tentative_gScore + H(neighbor));
					if (!openSet.contains(neighbor)) {
						openSet.add(neighbor);
					}
				}
			}

		}

		return null;

	}

	protected double getGScore(CurvePoint c){
		if (gScore.containsKey(c)) {
			return Objects.requireNonNull(gScore.get(c));
		}
		return REASONABLE_INFINITY;
	}

	protected double getFScore(CurvePoint c){
		if (fScore.containsKey(c)) {
			return Objects.requireNonNull(fScore.get(c));
		}
		return REASONABLE_INFINITY;
	}

	public void addCircleObstacle(double x, double y, double radius) {
		CurvePoint circleOrigin = new CurvePoint(x,y);
		for (CurvePoint[] row: field) {
			for (CurvePoint point:row) {
				if (point.distanceTo(circleOrigin) < radius) {
					point.setObstacle(true);
				}
			}
		}
		obstacles.add(new Circle(x,y,radius));
	}

	protected ArrayList<CurvePoint> reconstructPath(HashMap<CurvePoint, CurvePoint> cameFrom, CurvePoint current) {
		ArrayList<CurvePoint> reversedPath = new ArrayList<>();
		reversedPath.add(current);
		CurvePoint c = new CurvePoint(current);
		while (cameFrom.containsKey(c)) {
			c = new CurvePoint(Objects.requireNonNull(cameFrom.get(c)));
			reversedPath.add(c);
		}
		// now the path is the correct direction
		reverse(reversedPath);
		return reversedPath;
	}

	/**
	 * in place on arraylist of curve points will make the list followble by the controller
	 * @param path array list of curve points
	 */
	public void makePathStable(ArrayList<CurvePoint> path) {

		int slowDistance = 20;

		double slowestSpeed = 0.4;
		double maxSpeed = 1;


		// leaving a gap before the last point allows for better stability
		// this is the number of points we take out before the end to accomplish this
		int num_to_remove = 8;

		ArrayList<CurvePoint> pointsToRemove = new ArrayList<>();
		if (path.size() > num_to_remove + 2) {
			for (int i = path.size() - 1 - num_to_remove; i < path.size() - 1; ++i) {
				pointsToRemove.add(path.get(i));
			}
			for (CurvePoint point : pointsToRemove) {
				path.remove(point);
			}
		}

		// if the length of the path is less than 2 n different units, just make the whole thing at a low power
		if (path.size() < slowDistance * 2) {
			for (CurvePoint point: path) {
				point.moveSpeed = slowestSpeed;
			}
			return;
		}

		double ramp_diff = maxSpeed - slowestSpeed;
		double delta = ramp_diff / slowDistance;
		// ramp up the first n units
		for (int i = 0; i < slowDistance; ++i) {
			path.get(i).moveSpeed = slowestSpeed + delta * i;
		}

		// ramp down the last n units
		for (int i = path.size() - slowDistance; i < path.size(); ++i) {
			int zeroedIndex = i - path.size() + slowDistance + 1;
			System.out.println("zeroedIndex in ramp down is " + zeroedIndex + " i is " + i + " and path.size() is" + path.size());
			path.get(i).moveSpeed = 1 -  delta * zeroedIndex;
			path.get(i).followDistance = 3;
		}


	}
}
