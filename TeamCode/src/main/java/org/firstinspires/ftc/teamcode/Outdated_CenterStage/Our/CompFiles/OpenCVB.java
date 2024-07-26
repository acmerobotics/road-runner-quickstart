package org.firstinspires.ftc.teamcode.Outdated_CenterStage.Our.CompFiles;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class OpenCVB implements VisionProcessor {


    Rect Middle;
    Rect Right;

    Mat hsvMat= new Mat();
    Mat highMat = new Mat();
    Mat lowMat = new Mat();
    Mat DetectMat = new Mat();


    Scalar lowVRedlow = new Scalar(200,125,125);
    Scalar lowVRedHigh = new Scalar(250,255,255);

    Scalar upperVRedlow = new Scalar(200,125,125);
    Scalar upperVRedHigh = new Scalar(250,255,255);

    double MinThreshold = .00000001;



    PropLoc propLoc = PropLoc.Left;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

        Middle = new Rect(
                new Point(0,0),
                new Point(.5*width,.5*height)
        );
        Right = new Rect(
                new Point(.5*width,0),
                new Point(width,height)
        );

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        Imgproc.cvtColor(frame,hsvMat,Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsvMat,lowVRedlow,lowVRedHigh,lowMat);
        Core.inRange(hsvMat,upperVRedlow,upperVRedHigh,highMat);

        Core.bitwise_or(lowMat,highMat,DetectMat);

        double middlePercent = (Core.sumElems(DetectMat.submat(Middle)).val[1]/255)/Middle.area();
        double rightPercent = (Core.sumElems(DetectMat.submat(Right)).val[1]/255)/Right.area();


        Scalar green =new Scalar(0,250,0);
        Scalar red =new Scalar(250,0,0);

        if(middlePercent>rightPercent&& middlePercent>MinThreshold) {
            propLoc= PropLoc.Middle;
            Imgproc.rectangle(frame,Middle,green);
            Imgproc.rectangle(frame,Right,red);
        } else if (middlePercent<rightPercent && rightPercent>MinThreshold) {
            propLoc= PropLoc.Right;
            Imgproc.rectangle(frame,Right,green);
            Imgproc.rectangle(frame,Middle,red);
        }
        else if(middlePercent==0 && rightPercent==0) {
            Imgproc.rectangle(frame,Right,red);
            Imgproc.rectangle(frame,Middle,red);
        }


        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }

    public PropLoc getPropLoc (){
        return propLoc;
    }
    public enum PropLoc {
        Left(0),
        Middle(1),
        Right(2);

        public final int posNum;

        PropLoc(int posNum) {
            this.posNum = posNum;
        }
    }


}
