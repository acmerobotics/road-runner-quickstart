package org.firstinspires.ftc.teamcode.drive.opmode.visionIntoTheDeep;

import static java.lang.Thread.sleep;

import org.opencv.core.*;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.List;
import java.util.ArrayList;

public class itdBlobPipeline extends OpenCvPipeline {

    private final Mat grayscale = new Mat();
    private final Mat binary = new Mat();
    public boolean cameraBlocked;
    Mat hierarchy = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, grayscale, Imgproc.COLOR_RGB2GRAY);
        //converts image to grayscale

        Imgproc.threshold(grayscale,binary,150,200,Imgproc.THRESH_BINARY);
        // Create binary image with exclusively black/white. Simplifies reading the image later on.

        List<MatOfPoint> contours = new ArrayList();
        Imgproc.findContours(binary, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        //Looks for contours in the binary image.

        for (MatOfPoint contour : contours) {

            double area = Imgproc.contourArea(contour);
            // Calculate the areas

            double perimeter = Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true);
            // Calculate perimeters


            if (perimeter > 0) { // Avoids Dividing by 0
                double csf = (4 * Math.PI * area) / (perimeter * perimeter);
                // csf stands for circular shape factor, this helps us figure out if the shape we are analyzing is a polygon or not.

                if (csf < 0.7) {
                    Scalar color = new Scalar(0, 255, 0);
                    Imgproc.drawContours(input, List.of(contour), -1, color, 2);
                    // Draws line on output image so we can see what the system is identifying.

                    Moments moments = Imgproc.moments(contour);
                    int cx = (int) (moments.m10 / moments.m00);
                    int cy = (int) (moments.m01 / moments.m00);
                    // Grab center of centroid so we can place csf value in center for diagnostic purposes.

                    String getCSF = String.format("CSF: %.2f", csf);
                    Imgproc.putText(binary, getCSF, new Point(cx, cy), Imgproc.FONT_HERSHEY_SIMPLEX, 3.0, new Scalar(255.0, 0.0, 255.0));
                    // Prints CSF values on objects

                    if (area < 40000) {
                        cameraBlocked = true;
                    } else {
                        cameraBlocked = false;
                    }
                }
            }
        }
        grayscale.release();
        hierarchy.release();
        //release resources

        return binary;
    }

}