package org.firstinspires.ftc.teamcode.drive.opmode.visionIntoTheDeep;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class itdCameraPipeline extends OpenCvPipeline {

    private final Mat grayscale = new Mat();
    //This is the mat that receives the camera output after a Grayscale filter is applied

    private final Mat clean = new Mat();
    //Diagnostic matrix to adjust camera.


    public enum sampleAngleStage {
    farLeft,
    middleLeft,
    nearLeft,
    Center,
    nearRight,
    middleRight,
    farRight,
    angleUNKNOWN,
    //Creating enum's to pull from later for various functions
    }

    private Mat farLeftMat = new Mat();
    private Mat middleLeftMat = new Mat();
    private Mat nearLeftMat = new Mat();
    private Mat centerMat = new Mat();
    private Mat nearRightMat = new Mat();
    private Mat middleRightMat = new Mat();
    private Mat farRightMat = new Mat();
    //Individual matrices to detect how far left/right on the camera the sample is,
    // and by proxy what angle we need to turn the robot to face the sample head on.

    private double farLeftAvg, middleLeftAvg, nearLeftAvg, centerAvg, nearRightAvg, middleRightAvg, farRightAvg;
    //Collects the average of how many pixels have turned white after grayscale filter is applied.

    private final Rect farLeftRect = new Rect(0,0,183,720);
    private final Rect middleLeftRect = new Rect(183,0,183,720);
    private final Rect nearLeftRect = new Rect(366,0,183,720);
    private final Rect centerRect = new Rect(548,0,183,720);
    private final Rect nearRightRect = new Rect(731,0,183,720);
    private final Rect middleRightRect = new Rect(914,0,183,720);
    private final Rect farRightRect = new Rect(1097,0,183,720);


    public enum sampleHeightStage {
        A,
        B,
        C,
        D,
        E,
        F,
        G,
        heightUNKNOWN,
    }
    // Creating bars to locate sample positions

    private Mat aMat = new Mat();
    private Mat bMat = new Mat();
    private Mat cMat = new Mat();
    private Mat dMat = new Mat();
    private Mat eMat = new Mat();
    private Mat fMat = new Mat();
    private Mat gMat = new Mat();
    //Individual matrices to detect how high on the camera the sample is,
    // and by proxy how far ahead of the robot it is.
    private double aAvg, bAvg, cAvg, dAvg, eAvg, fAvg, gAvg;
    //Collects the average of how many pixels have turned white after grayscale filter is applied.

    private final Rect aRect = new Rect(0,0,1280,103);
    private final Rect bRect = new Rect(0,104,1280,103);
    private final Rect cRect = new Rect(0,207,1280,103);
    private final Rect dRect = new Rect(0,310,1280,101);
    private final Rect eRect = new Rect(0,411,1280,103);
    private final Rect fRect = new Rect(0,514,1280,103);
    private final Rect gRect = new Rect(0,617,1280,103);

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, grayscale, Imgproc.COLOR_RGB2GRAY);
        //converts image to grayscale
        input.copyTo(grayscale);

        aAvg = calculateAverageIntensity(grayscale.submat(aRect));
        bAvg = calculateAverageIntensity(grayscale.submat(bRect));
        cAvg = calculateAverageIntensity(grayscale.submat(cRect));
        dAvg = calculateAverageIntensity(grayscale.submat(dRect));
        eAvg = calculateAverageIntensity(grayscale.submat(eRect));
        fAvg = calculateAverageIntensity(grayscale.submat(fRect));
        gAvg = calculateAverageIntensity(grayscale.submat(gRect));

        // Calculate averages for angle regions
        farLeftAvg = calculateAverageIntensity(grayscale.submat(farLeftRect));
        middleLeftAvg = calculateAverageIntensity(grayscale.submat(middleLeftRect));
        nearLeftAvg = calculateAverageIntensity(grayscale.submat(nearLeftRect));
        centerAvg = calculateAverageIntensity(grayscale.submat(centerRect));
        nearRightAvg = calculateAverageIntensity(grayscale.submat(nearRightRect));
        middleRightAvg = calculateAverageIntensity(grayscale.submat(middleRightRect));
        farRightAvg = calculateAverageIntensity(grayscale.submat(farRightRect));

        /*Imgproc.rectangle(grayscale, aRect, new Scalar(255.0, 255.0, 0.0), 2);
        Imgproc.rectangle(grayscale, bRect, new Scalar(255.0, 255.0, 0.0), 2);
        Imgproc.rectangle(grayscale, cRect, new Scalar(255.0, 255.0, 0.0), 2);
        Imgproc.rectangle(grayscale, dRect, new Scalar(255.0, 255.0, 0.0), 2);
        Imgproc.rectangle(grayscale, eRect, new Scalar(255.0, 255.0, 0.0), 2);
        Imgproc.rectangle(grayscale, fRect, new Scalar(255.0, 255.0, 0.0), 2);
        Imgproc.rectangle(grayscale, gRect, new Scalar(255.0, 255.0, 0.0), 2);

        Imgproc.rectangle(grayscale, farLeftRect, new Scalar(255.0,0.0,255.0),2);
        Imgproc.rectangle(grayscale, middleLeftRect, new Scalar(255.0,0.0,255.0),2);
        Imgproc.rectangle(grayscale, nearLeftRect, new Scalar(255.0,0.0,255.0),2);
        Imgproc.rectangle(grayscale, centerRect, new Scalar(255.0,0.0,255.0),2);
        Imgproc.rectangle(grayscale, nearRightRect, new Scalar(255.0,0.0,255.0),2);
        Imgproc.rectangle(grayscale, middleRightRect, new Scalar(255.0,0.0,255.0),2);
        Imgproc.rectangle(grayscale, farRightRect, new Scalar(255.0,0.0,255.0),2);*/
        // Creates rectangles on the driver hub vision screen for each matrix to better diagnose issues,
        // and visualize what stage is active.

        double[] angleAverages = {farLeftAvg, middleLeftAvg, nearLeftAvg, centerAvg, nearRightAvg, middleRightAvg, farRightAvg};
        String[] angle = {"farLeft", "middleLeft", "nearLeft", "center", "nearRight", "middleRight", "farRight"};
        // converts the avg's into strings to be analyzed in the integar array below

        int minAngleIndex = 0;
        for (int i = 1; i < angleAverages.length; i++) {
            if (angleAverages[i] < angleAverages[minAngleIndex]) {
                minAngleIndex = i;
            }
        }

        double[] sampleHeightAverages = {aAvg, bAvg, cAvg, dAvg, eAvg, fAvg, gAvg};
        String[] sampleHeight = {"A", "B", "C", "D", "E", "F", "G"};
        // converts the avg into strings to be analyzed in the integer array below

        int minHeightIndex = 0;
        for (int i = 1; i < sampleHeightAverages.length; i++) {
            if (sampleHeightAverages[i] < sampleHeightAverages[minHeightIndex]) {
                minHeightIndex = i;
            }
        }
        // the index starts at zero assuming whatever aAvg's value is as the minimum, and resets each time it gathers a new minimum
        // the for loop goes over all the averages checking if that average is lower than the previous minimum found
        // the loop terminates at G and will start over again to return a new value.


        String getAngle = angle[minAngleIndex];
        String getSampleHeight = sampleHeight[minHeightIndex];

        Imgproc.putText(grayscale, getSampleHeight, new Point(25, 100), Imgproc.FONT_HERSHEY_SIMPLEX, 3.0, new Scalar(255.0, 255.0, 0.0));
        // displays the measured Sample Location on screen.
        Imgproc.putText(grayscale, getAngle, new Point(100, 100), Imgproc.FONT_HERSHEY_SIMPLEX,3.0, new Scalar(255.0,0.0,255.0));
        // displays the measured angle on screen.
        return grayscale;
    }
    private double calculateAverageIntensity(Mat region) {
        Scalar avgIntensity = Core.mean(region);
        return avgIntensity.val[0];
    }}