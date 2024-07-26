package org.firstinspires.ftc.teamcode.Kotlin_Bromine_Arya.Opmodes.Subsystems.CV;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class CV_Pipeline implements VisionProcessor {
    Location location = Location.none;
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar RED = new Scalar(255, 0, 0);
    double cameraWidth;
    double cameraHeight;
    final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0,0);
    final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(cameraWidth/3, 0);
    final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(2*cameraWidth/3,0);
    final double RegionWidth = cameraWidth/3;
    final double RegionHeight = cameraHeight;
    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + RegionWidth,
            REGION1_TOPLEFT_ANCHOR_POINT.y + RegionHeight);
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + RegionWidth,
            REGION2_TOPLEFT_ANCHOR_POINT.y + RegionHeight);
    Point region3_pointA = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x,
            REGION3_TOPLEFT_ANCHOR_POINT.y);
    Point region3_pointB = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x + RegionWidth,
            REGION3_TOPLEFT_ANCHOR_POINT.y + RegionHeight);

    Mat Left_Rect,Middle_Rect, Right_Rect;
    Mat HSV_Mat = new Mat();
    @Override
    public void init(int width, int height, CameraCalibration calibration){
        cameraWidth=width;
        cameraHeight=height;

        Left_Rect = HSV_Mat.submat(new Rect(region1_pointA,region1_pointB));
        Middle_Rect = HSV_Mat.submat(new Rect(region2_pointA,region2_pointB));
        Right_Rect = HSV_Mat.submat(new Rect(region3_pointA,region3_pointB));
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos) {
        Imgproc.cvtColor(input,HSV_Mat,Imgproc.COLOR_RGB2HSV);

        double avg1, avg2,avg3;
        avg1= (int) Core.mean(Left_Rect).val[0];
        avg2= (int) Core.mean(Middle_Rect).val[0];
        avg3= (int) Core.mean(Right_Rect).val[0];

        Imgproc.rectangle(input, region1_pointA, region1_pointB, BLUE, 2);
        Imgproc.rectangle(input, region2_pointA, region2_pointB, BLUE, 2);
        Imgproc.rectangle(input, region3_pointA, region3_pointB, BLUE, 2);

        double minorMax = Math.max(avg1,avg2);
        double max = Math.max(minorMax,avg3);

        if(max == avg1){
            location = Location.left;
            Imgproc.rectangle(input, region1_pointA, region1_pointB, RED, 2);
        } else if (max == avg2) {
            location = Location.middle;
            Imgproc.rectangle(input, region2_pointA, region2_pointB, RED, 2);
        } else if (max==avg3) {
            location = Location.right;
            Imgproc.rectangle(input, region3_pointA, region3_pointB, RED, 2);
        }

        //Destroy mats after each iteration
        Left_Rect.release();
        Middle_Rect.release();
        Right_Rect.release();
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
    public Location getLocation(){
        return location;
    }
    public enum Location {
        left, middle,right,none
    }

}
