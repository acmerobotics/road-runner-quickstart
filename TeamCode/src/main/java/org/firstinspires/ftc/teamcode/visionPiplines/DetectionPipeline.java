package org.firstinspires.ftc.teamcode.visionPiplines;




import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;


@Config
public class DetectionPipeline extends OpenCvPipeline {


    public static Scalar upper = new Scalar(190,210,150,255);
    public static Scalar lower = new Scalar(100,140,60,0);

    protected Pole pole = new Pole();

    double minRatio = 500;


    private ArrayList<Mat> matsToRelease = new ArrayList<>();


    public DetectionPipeline() {
    }




    @Override
    public Mat processFrame(Mat input) {

        fuckMemoryLeaks();

        Mat mask = new Mat();
        Mat out = new Mat();
        Imgproc.cvtColor(input,input,Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(input,lower,upper,mask);
        Core.bitwise_and(input,input,out,mask);
        Mat blackAndWhite = new Mat();
        // Imgproc.cvtColor(out, blackAndWhite, Imgproc.COLOR_YCrCb2BGR);
        Imgproc.cvtColor(out, blackAndWhite, Imgproc.COLOR_BGR2GRAY);


        ArrayList<MatOfPoint> detectedContours = new ArrayList<>();

        Imgproc.findContours(blackAndWhite,detectedContours,new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);


        ArrayList<MatOfPoint> contours_to_remove = new ArrayList<>();
        if (detectedContours.size() < 1) {
            return out;
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




        Imgproc.drawContours(input,detectedContours,-1, new Scalar(0.0, 255.0, 0.0), 3);
        roundupMemory(blackAndWhite,out,mask);




        return input;
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

    @NonNull
    public Pole getPole() {
        return pole;
    }
}