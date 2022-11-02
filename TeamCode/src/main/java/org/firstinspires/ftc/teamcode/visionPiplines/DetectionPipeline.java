package org.firstinspires.ftc.teamcode.visionPiplines;




import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;


public class DetectionPipeline extends OpenCvPipeline {


    public static Scalar upper_1 = new Scalar(190,210,150,255);
    public static Scalar lower_1 = new Scalar(255,255,255,0);

    public static Scalar upper_2 = new Scalar(28.8,176.9,255,255);
    public static Scalar lower_2 = new Scalar(9.6,26.0,137.1,0);


    protected Pole pole = new Pole();

    double minRatio = 500;


    private ArrayList<Mat> matsToRelease = new ArrayList<>();


    public DetectionPipeline() {}


    @Override
    public Mat processFrame(Mat input) {

        fuckMemoryLeaks();

        Mat mask = new Mat();
        Mat mask_2 = new Mat();
        Mat out = new Mat();
        Mat converted = new Mat();
        Mat converted_HSV = new Mat();
        Imgproc.cvtColor(input,converted,Imgproc.COLOR_RGB2YCrCb);
        Imgproc.cvtColor(input,converted_HSV,Imgproc.COLOR_RGB2HSV);
        Core.inRange(converted,lower_1,upper_1,mask);
        Core.inRange(converted_HSV,lower_2,upper_2,mask_2);
        Core.add(mask,mask_2,mask);
        Core.bitwise_and(converted,converted,out,mask);

        Mat blackAndWhite = new Mat();
        // Imgproc.cvtColor(out, blackAndWhite, Imgproc.COLOR_YCrCb2BGR);
        Imgproc.cvtColor(out, blackAndWhite, Imgproc.COLOR_BGR2GRAY);


        ArrayList<MatOfPoint> detectedContours = new ArrayList<>();
        Imgproc.medianBlur(blackAndWhite,blackAndWhite,5);
        Imgproc.findContours(blackAndWhite,detectedContours,new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);


        ArrayList<MatOfPoint> contours_to_remove = new ArrayList<>();
        if (detectedContours.size() < 1) {
            return input;
        } else {
            // find largest height to width ratio

            int largestRatio = detectedContours.get(0).height() / detectedContours.get(0).width();
            MatOfPoint currentContour = detectedContours.get(0);

            if (largestRatio < minRatio) {
                largestRatio = 0;
                contours_to_remove.add(currentContour);
            }


            for (MatOfPoint c:detectedContours) {
                int currentRatio = c.height() / c.width();
                if (currentRatio > largestRatio && minRatio < currentRatio) {
                    largestRatio = currentRatio;
                    contours_to_remove.add(currentContour);
                    currentContour = c;

                } else {
                    contours_to_remove.add(c);
                }
            }
            Imgproc.drawContours(converted,detectedContours,-1, new Scalar(255.0, 255.0, 255.0), 3);

            for (MatOfPoint c: contours_to_remove) {
                detectedContours.remove(c);
            }

            if (detectedContours.size() > 0) {
                Rect boundingBox = Imgproc.boundingRect(detectedContours.get(0));
                int pos = boundingBox.x + boundingBox.width/2;
                int width = boundingBox.width;
                this.pole = new Pole(pos,width);
            } else {
                this.pole = new Pole();
            }

            System.out.println("detected Pole: " + this.pole);

        }

        Imgproc.drawContours(converted,detectedContours,-1, new Scalar(0, 255.0, 0.0), 3);
        roundupMemory(blackAndWhite,out,mask,mask_2,converted,converted_HSV);

        return converted;
    }



    public void fuckMemoryLeaks() {
        for (Mat m: matsToRelease) {
            m.release();
        }
    }
    public void roundupMemory(Mat... Mats) {
        matsToRelease.addAll(Arrays.asList(Mats));
    }

    @Override
    public void onViewportTapped() {
    }

    public Pole getPole() {
        return pole;
    }
}