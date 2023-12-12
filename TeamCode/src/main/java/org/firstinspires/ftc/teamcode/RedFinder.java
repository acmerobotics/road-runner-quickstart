package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

public class RedFinder implements VisionProcessor {

    final static int RESOLUTION_WIDTH = 1920;
    final static int RESOLUTION_HEIGHT = 1080;

    public static int LW_LEFT = 300;
    public static int LW_TOP = 400;
    public static int LW_WIDTH = 340;
    public static int LW_HEIGHT = 500 ;

    public static int CW_LEFT=800;
    public static int CW_TOP=400;
    public static int CW_WIDTH= 580;
    public static int CW_HEIGHT= 300;


    public static int RW_LEFT = 1460;
    public static int RW_TOP = 400;
    public static int RW_WIDTH = 350 ;
    public static int RW_HEIGHT = 500;

    public static double THRESHOLD = 5.0;

    // JRC: These are switched?!
    public final int BLUE_CHANNEL = 1;
    public final int RED_CHANNEL = 2;
    public String side = "None";            // accessible side ID
    public double leftMean, centerMean, rightMean;        // accessible mean red metrics
    // if the mean red of one side is greater than THRESHOLD + mean of the other, that side is "Red"
    //public static double THRESHOLD = 5.0;
    public Boolean INPUT = true;
    public Boolean OUTPUT = false;
    Boolean inOut = INPUT;
    Rect leftWindow, centerWindow, rightWindow;

    public String getSide() {
        return side;
    }

    @java.lang.Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @java.lang.Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (frame == null) {
            return null;
        }
        Mat workingMat = new Mat();

        // Pretty sure this should be COLOR_RGB2YCrCb
        Imgproc.cvtColor(frame, workingMat, Imgproc.COLOR_BGR2YCrCb);   // Possibly: Use HSV.  Use inRange() to convert to binary matrix.

        if (LW_TOP + LW_HEIGHT >= workingMat.height()) {
            LW_HEIGHT = workingMat.height() - LW_TOP - 1;
        }
        if (LW_LEFT + LW_WIDTH >= workingMat.width()) {
            LW_WIDTH = workingMat.width() - LW_LEFT - 1;
        }
        if (CW_TOP + CW_HEIGHT >= workingMat.height()) {
            CW_HEIGHT = workingMat.height() - CW_TOP - 1;
        }
        if (CW_TOP + CW_HEIGHT >= workingMat.height()) {
            CW_WIDTH = workingMat.width() - CW_LEFT - 1;
        }

        if (RW_TOP + RW_HEIGHT >= workingMat.height()) {
            RW_HEIGHT = workingMat.height() - RW_TOP - 1;
        }
        if (RW_LEFT + RW_WIDTH >= workingMat.width()) {
            RW_WIDTH = workingMat.width() - RW_LEFT - 1;
        }

        leftWindow = new Rect(LW_LEFT, LW_TOP, LW_WIDTH, LW_HEIGHT);
        centerWindow = new Rect(CW_LEFT, LW_TOP, CW_WIDTH, CW_HEIGHT);
        rightWindow = new Rect(RW_LEFT, RW_TOP, RW_WIDTH, RW_HEIGHT);

        Mat leftCrop = workingMat.submat(leftWindow);
        Core.extractChannel(leftCrop, leftCrop, RED_CHANNEL);

        Mat centerCrop = workingMat.submat(centerWindow);
        Core.extractChannel(centerCrop, centerCrop, RED_CHANNEL);

        Mat rightCrop = workingMat.submat(rightWindow);
        Core.extractChannel(rightCrop, rightCrop, RED_CHANNEL);

        leftMean = Core.mean(leftCrop).val[0];
        centerMean = Core.mean(centerCrop).val[0];
        rightMean = Core.mean(rightCrop).val[0];

        if (leftMean > rightMean && leftMean > centerMean) {
            side = "Left";
        } else if (centerMean > leftMean && centerMean > rightMean) {
            side = "Center";
        } else {
            side = "Right";
        }

// release all created matrices!  Otherwise a memory leak will occur.
        leftCrop.release();
        centerCrop.release();
        rightCrop.release();
        if (inOut) {
            workingMat.release();
            return frame;
        } else {
            frame.release();
            return workingMat;
        }
    }

    @java.lang.Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        Paint leftPaint = new Paint();
        if (side.equals("Left")) {
            leftPaint.setColor(Color.RED);
        } else {
            leftPaint.setColor(Color.GREEN);
        }
        leftPaint.setStyle(Paint.Style.STROKE);
        leftPaint.setStrokeWidth(scaleCanvasDensity * 4);
        canvas.drawRect(makeGraphicsRect(leftWindow, scaleBmpPxToCanvasPx), leftPaint);

        Paint centerPaint = new Paint();
        if (side.equals("Center")) {
            centerPaint.setColor(Color.RED);
        } else {
            centerPaint.setColor(Color.GREEN);
        }
        centerPaint.setStyle(Paint.Style.STROKE);
        centerPaint.setStrokeWidth(scaleCanvasDensity * 4);
        canvas.drawRect(makeGraphicsRect(centerWindow, scaleBmpPxToCanvasPx), centerPaint);


        Paint rightPaint = new Paint();
        if (side.equals("Right")) {
            rightPaint.setColor(Color.RED);
        } else {
            rightPaint.setColor(Color.GREEN);
        }
        rightPaint.setStyle(Paint.Style.STROKE);
        rightPaint.setStrokeWidth(scaleCanvasDensity * 4);

        canvas.drawRect(makeGraphicsRect(rightWindow, scaleBmpPxToCanvasPx), rightPaint);


    }
    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = Math.round(left + rect.width * scaleBmpPxToCanvasPx);
        int bottom = Math.round(top + rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }
}
