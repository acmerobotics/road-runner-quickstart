package org.firstinspires.ftc.teamcode.SummerTrainingCourse.CVproject;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class Pipeline implements VisionProcessor {

    public enum Location{
        Left(0),Middle(1),Right(2),None(100);
        private final int value;
        Location(int value){
            this.value = value;
        }
    }
    public enum Color{
        Blue,Red
    }
    public Location location = Location.None;
    public Color color = null;

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
        //submaqts

        Left_rect = HSv_Mat.submat(new Rect(region1_pointA,region1_pointB));
        Middle_rect = HSv_Mat.submat(new Rect(region2_pointA,region2_pointB));
        Right_Rect = HSv_Mat.submat(new Rect(region3_pointA,region3_pointB));

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame,HSv_Mat,Imgproc.COLOR_RGB2HSV);

        double [][] averages;

        double [] avg1 = new double[]{Core.mean(Left_rect).val[0], Core.mean(Left_rect).val[1], Core.mean(Left_rect).val[2]};
        double [] avg2 = new double[]{Core.mean(Middle_rect).val[0], Core.mean(Middle_rect).val[1], Core.mean(Middle_rect).val[2]};
        double [] avg3 = new double[]{Core.mean(Right_Rect).val[0], Core.mean(Right_Rect).val[1], Core.mean(Right_Rect).val[2]};

        averages = new double[][]{avg1,avg2,avg3};

        switch (color){
            case Red:{
                double upUpHue = 360, upLowHue = 330, lowLowHue = 0, lowUpHue = 30;

                for(Location i: Location.values())
                    if ((averages[i.value][0] > upLowHue && averages[i.value][0] < upUpHue) || (averages[i.value][0] > lowLowHue && averages[i.value][0] < lowUpHue)) {
                        location = i;
                        break;
                    }
            }
            case Blue:{
                double upHue = 70, lowHue = 90;
                for(Location i: Location.values())
                    if (averages[i.value][0] > lowHue && averages[i.value][0] < upHue) {
                        location = i;
                        break;
                    }
            }

        }
        return null;
    }

    public Location getLocation(){
        return location;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }
}
