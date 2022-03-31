package org.firstinspires.ftc.teamcode.hardware.util;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
@Config
public class kellen extends OpenCvPipeline {
    //pipe line class for detecting capstone (or objects in three different regions)
    //default color detections set to lime green
    public kellen() {
        color = "red";
    }
    public kellen(String choice) {
        color = choice;
    }
    boolean viewportPaused;

    private Mat workingMatrix = new Mat();
    private Scalar lowHSV;
    private Scalar highHSV;
    private double b1p, b2p, b3p;
    private String color;
    private Rect RO1;
    private Rect RO2;
    private Rect RO3;

    @Override
    public final Mat processFrame(Mat input){
        //cvtColor converts a feed from a certain color format to another color format; in this case, we are converting from RGB to HSV
        Imgproc.cvtColor(input, workingMatrix, Imgproc.COLOR_RGB2HSV);

        /*lowHSV and highHSV are our thresholds
        IMPORTANT NOTE: openCV defines HSV parameters as such (Hue, Saturation, Value) where Hue is Range 0-179,
        Saturation is in range 0-255 and Value is in range 0-255 (all INCLUSIVE).
        NORMALLY, HSV is defined like so: Hue in range 0-360, Saturation in range 0.0-1.0 and Value in range 0.0-1.0.
        All of this is also technically BGR to HSV, so take the absolute value of your Hue minus 180 to get the right number
        */
        if(color == "green") {
            lowHSV = new Scalar(40, 50, 50);
            highHSV = new Scalar(75, 255, 255);
        }
        if(color == "red") {
            lowHSV = new Scalar(160, 50, 50);
            highHSV = new Scalar(180, 255, 255);
        }
        if(color == "blue") {
            lowHSV = new Scalar(110, 50, 50);
            highHSV = new Scalar(120, 255, 255);
        }
        if(color == "purple") {
            lowHSV = new Scalar(135, 50, 50);
            highHSV = new Scalar(155, 255, 255);
        }
        if(color == "cyan") {
            lowHSV = new Scalar(80, 50, 50);
            highHSV = new Scalar(95, 255, 255);
        }
        if(color == "yellow") {
            lowHSV = new Scalar(20, 55, 55);
            highHSV = new Scalar(35, 255, 255);
        }
        if(color == "orange") {
            lowHSV = new Scalar(0, 50, 50);
            highHSV = new Scalar(20, 255, 255);
        }

        //This creates our mask, and filters out all colors except for whats within our defined bound
            /* IGNORE ALL OF THIS FOR NOW, but essentially we'll use this to tell where our capstone is by counting pixels

            creates the submat that we want to work with
            Mat region = workingMatrix.submat(ROI);
            //this counts the number of white pixels and divides it by the area of our ROI to figure out the percentage.
            regionValue = Core.sumElems(region).val[0] / ROI.area()/255;
            //you need to release the channel that we worked with or smthn smhtn; in this case we have to release region for some reason
            region.release();
            //line color
            Scalar lines = new Scalar(25,255,255);
            //Create the rectangle so that when testing we can see the ROI that we are working with
            Imgproc.rectangle(workingMatrix,ROI,lines);*/
        RO1 = new Rect(
                new Point(
                        input.cols()/8,
                        input.rows()/4),
                new Point(
                        input.cols()*(3f/8f),
                        input.rows()*(3f/4f))
        );
        RO2 = new Rect(
                new Point(
                        input.cols()*(3f/8f),
                        input.rows()/4),
                new Point(
                        input.cols()*(5f/8f),
                        input.rows()*(3f/4f))
        );
        RO3 = new Rect(
                new Point(
                        input.cols()*(5f/8f),
                        input.rows()/4),
                new Point(
                        input.cols()*(7f/8f),
                        input.rows()*(3f/4f))
        );
        /*Imgproc.rectangle(workingMatrix, RO1, new Scalar(60, 255, 255), 10);
        Imgproc.rectangle(workingMatrix, RO2, new Scalar(60, 255, 255), 10);
        Imgproc.rectangle(workingMatrix, RO3, new Scalar(60, 255, 255), 10);*/
        Core.inRange(workingMatrix, lowHSV, highHSV, workingMatrix);

        //Submats for boxes, these are the regions that'll detect the color

        Mat box1 = workingMatrix.submat(RO1);
        Mat box2 = workingMatrix.submat(RO2);
        Mat box3 = workingMatrix.submat(RO3);
        //How much in each region is white aka the color we filtered
        b1p = Core.sumElems(box1).val[0] / RO1.area()/255;
        b2p = Core.sumElems(box2).val[0] / RO2.area()/255;
        b3p = Core.sumElems(box3).val[0] / RO3.area()/255;
        //Compare amount of color in each region
        if(b1p > b2p && b1p > b3p) {
            Imgproc.rectangle(workingMatrix, RO1, new Scalar(60, 255, 255), 10);
        }else if(b2p > b1p && b2p > b3p) {
            Imgproc.rectangle(workingMatrix, RO2, new Scalar(60, 255, 255), 10);
        }else if(b3p > b2p & b3p > b1p) {
            Imgproc.rectangle(workingMatrix, RO3, new Scalar(60, 255, 255), 10);
        }
        //return the frame
        return workingMatrix;
    }

    public double region1percent() {
        return b1p * 100;
    }

    public double region2percent() {
        return b2p * 100;
    }

    public double region3percent() {
        return b3p * 100;
    }

    public int whichRegion() {
        int region = 0;
        if(b1p > b2p && b1p > b3p) {
            region = 1;
        }else if(b2p > b1p && b2p > b3p) {
            region = 2;
        }else if(b3p > b2p & b3p > b1p) {
            region = 3;
        }
        return region;
    }
}
