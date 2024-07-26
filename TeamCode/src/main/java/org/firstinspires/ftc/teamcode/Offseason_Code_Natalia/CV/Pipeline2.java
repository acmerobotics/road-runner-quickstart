package org.firstinspires.ftc.teamcode.Offseason_Code_Natalia.CV;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.SummerTrainingCourse.CVproject.Pipeline;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

//this is for detection of RED, i didn't feel like adding both but I'll do it this week probably
//let's hope this works.

public class Pipeline2 implements VisionProcessor{
//    public enum{
//        Blue,
//        Red
//    }
//    private ColorMode colorMode;
//
//    // Constructor that takes ColorMode as a parameter
//    public (ColorMode mode) {
//        this.colorMode = mode;

    public Pipeline.Location location = Pipeline.Location.None;

    double cameraWidth = 1920;
    double cameraHeight = 1080;

    final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0,0);
    final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(cameraWidth/3,cameraHeight);
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

    Mat Left_rect, Middle_rect, Right_Rect;
    Mat HSv_Mat = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        //submats

        Left_rect = HSv_Mat.submat(new Rect(region1_pointA,region1_pointB));
        Middle_rect = HSv_Mat.submat(new Rect(region2_pointA,region2_pointB));
        Right_Rect = HSv_Mat.submat(new Rect(region3_pointA,region3_pointB));

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame,HSv_Mat,Imgproc.COLOR_RGB2HSV);

        double [] avg1;
        double [] avg2;
        double [] avg3;
        // these are the orange-red hues for this, I realize i need to do the top red that are more in the 0-360 magenta range
        //but i'm lazy so i'll do it later
        double LowerHueThreshold = 0;
        double UpperHueThreshold = 30;

        avg1 = new double[]{Core.mean(Left_rect).val[0], Core.mean(Middle_rect).val[1], Core.mean(Middle_rect).val[2]};
        avg2 = new double[]{Core.mean(Middle_rect).val[0], Core.mean(Middle_rect).val[1], Core.mean(Middle_rect).val[2]};
        avg3 = new double[]{Core.mean(Right_Rect).val[0], Core.mean(Right_Rect).val[1], Core.mean(Right_Rect).val[2]};

        if(avg1[0] > UpperHueThreshold && avg1[2] > 45){
            location = Pipeline.Location.Left;
        }

        if(avg2[0] > UpperHueThreshold && avg2[2] > 45){
            location = Pipeline.Location.Middle;
        }
        if(avg3[0] > UpperHueThreshold && avg3[2] > 45){
            location = Pipeline.Location.Right;
        }
//i  added detection for the other two sides as well

        //TODO(Set thresholds to different hues, depending on a seperate variable)

        //0-28 || 338-360




        return null;
    }


    public org.firstinspires.ftc.teamcode.SummerTrainingCourse.CVproject.Pipeline.Location getLocation(){
        return location;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }


    public enum Location{
        Left,Middle,Right,None
    }

}

