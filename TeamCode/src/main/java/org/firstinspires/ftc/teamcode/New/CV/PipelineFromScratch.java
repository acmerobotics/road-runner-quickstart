package org.firstinspires.ftc.teamcode.New.CV;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class PipelineFromScratch implements VisionProcessor {
    public enum Location {
        Left,
        Right,
        Middle,

    }

    Location left;

//        public PipelineFromScratch.Location location = PipelineFromScratch.location;

    int cameraWidth = 1920;
    int cameraHeight = 1080;
    private OpenCvCamera controlhubcamera;


    final Point BOTTOMRIGHT = new Point(1, 1);
    final Point BOTTOMLEFT = new Point(1919, 1);
    final Point TOPRIGHT = new Point(1, 1079);
    final Point TOPLEFT = new Point(1919, 1079);

    int RegionWidth = cameraWidth / 3;
    int RegionHeight = cameraHeight;

    Point region1topright = new Point(
            TOPRIGHT.x - RegionWidth,
            TOPRIGHT.y + 0);


    Point region3bottomleft = new Point(
            BOTTOMRIGHT.x - RegionWidth,
            BOTTOMRIGHT.y + 0);


    Point region3upperleft = new Point(
            TOPRIGHT.x - RegionHeight,
            TOPRIGHT.y + 0);


    public Mat region1;
    public Mat region2;
    public Mat region3;
    Mat HSvMat = new Mat();




    public void init(int width, int height, CameraCalibration calibration) {

        //submats


        region1 = HSvMat.submat(new Rect(BOTTOMLEFT, region1topright));
        region2 = HSvMat.submat(new Rect(region1topright, region3bottomleft));
        region3 = HSvMat.submat(new Rect(region1topright, BOTTOMLEFT));

    }

//    @Override
//    public Mat processFrame(Mat frame) {
//
//
//        //Imgproc;
//        double LowerHueThreshold = 0;
//        double UpperHueThreshold = 131;
//        double[] avg1;
//        double[] avg2;
//        double[] avg3;
//
//        Scalar RedHueThreshold;
//
//        avg1 = new double[]{Core.mean(region1).val[0]};
//        avg2 = new double[]{Core.mean(region2).val[0]};
//        avg3 = new double[]{Core.mean(region3).val[0]};
//
//        if (avg1[0] < UpperHueThreshold && avg1[0] > LowerHueThreshold) {
//
//
//        }
//        return null;
//
//
//    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Not useful either
    }

//        @Override
//        public void onViewportTapped() {
//            //screen is clicked or tapped
//
//



    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        double LowerHueThreshold = 0;
        double UpperHueThreshold = 131;
        double[] avg1;
        double[] avg3;
        double[] avg2;

        avg1 = new double[]{Core.mean(region1).val[0], Core.mean(region1).val[1], Core.mean(region1).val[2]};
        avg2 = new double[]{Core.mean(region2).val[0], Core.mean(region2).val[1], Core.mean(region2).val[2]};
        avg3 = new double[]{Core.mean(region3).val[0], Core.mean(region3).val[1], Core.mean(region3).val[2]};

        if (avg1[0] < UpperHueThreshold && avg1[0] > LowerHueThreshold) {
            left = Location.Left;
        }
        if (avg2[0] < UpperHueThreshold && avg1[0] > LowerHueThreshold){
            left = Location.Middle;
        }
        if (avg3[0] < UpperHueThreshold && avg1[0] > LowerHueThreshold){
            left = Location.Right;
        }





        return null;




    }



}










