package org.firstinspires.ftc.teamcode.auton;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.calib3d.Calib3d;
import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;

import java.util.ArrayList;
import java.util.List;

public class openCVHomography {

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    private Mat homographyMatrix;

    public openCVHomography() {
        homographyMatrix = calculateHomography(); // Initialize the homography matrix
    }

    public static Mat calculateHomography() {
        // Define four points in the source (image) coordinates
        List<Point> srcPoints = new ArrayList<>();
        srcPoints.add(new Point(100, 150)); // Example points, replace with actual values
        srcPoints.add(new Point(200, 150));
        srcPoints.add(new Point(200, 250));
        srcPoints.add(new Point(100, 250));

        // Define corresponding points in the destination (world) coordinates
        List<Point> dstPoints = new ArrayList<>();
        dstPoints.add(new Point(0, 0));     // Example world points
        dstPoints.add(new Point(1, 0));     //X is distance from left to right
        dstPoints.add(new Point(1, 1));     //Y is distance forward to the sample
        dstPoints.add(new Point(0, 1));

        // Convert lists to MatOfPoint2f
        MatOfPoint2f srcMat = new MatOfPoint2f();
        srcMat.fromList(srcPoints);

        MatOfPoint2f dstMat = new MatOfPoint2f();
        dstMat.fromList(dstPoints);

        // Calculate the homography matrix
        return Calib3d.findHomography(srcMat, dstMat);
    }

    public double[] convertTo3D(Point imagePoint) {
        // Create a point in homogeneous coordinates
        Mat srcPoint = Mat.ones(3, 1, CvType.CV_64F);
        srcPoint.put(0, 0, imagePoint.x);
        srcPoint.put(1, 0, imagePoint.y);

        // Calculate the destination point
        Mat dstPoint = new Mat();
        Core.gemm(homographyMatrix, srcPoint, 1, new Mat(), 0, dstPoint);

        // Convert back to Euclidean coordinates
        double w = dstPoint.get(2, 0)[0];
        double X = dstPoint.get(0, 0)[0] / w;
        double Y = dstPoint.get(1, 0)[0] / w;

        return new double[]{X, Y, 0}; // Return as an array [X, Y, Z]
    }

    public static void main(String[] args) {
        openCVHomography homographyCalculator = new openCVHomography();
        Mat homographyMatrix = homographyCalculator.homographyMatrix;
        System.out.println("Homography Matrix: \n" + homographyMatrix.dump());

        // Sample tx and ty from Limelight (replace with actual values from camera)
        double tx = 150; // Example tx
        double ty = 200; // Example ty

        Point imagePoint = new Point(tx, ty);
        double[] worldCoordinates = homographyCalculator.convertTo3D(imagePoint);
        System.out.println("3D World Coordinates: X=" + worldCoordinates[0] + ", Y=" + worldCoordinates[1] + ", Z=" + worldCoordinates[2]);
    }
}