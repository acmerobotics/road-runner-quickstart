package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


@Config
public class ConceptOpenCVHSV implements VisionProcessor {

    public static Boolean showOutput = false;   // for config purposes - can view output rather than input.
                                                // DOES NOT PLAY NICELY WITH OTHER VISION PROCESSORS THAT EXPECT THE INPUT STREAM


    Rect leftWindow, centerWindow, rightWindow;
    public static int LW_LEFT = 300;
    public static int LW_TOP = 400;
    public static int LW_RIGHT = 640;
    public static int LW_BOTTOM = 900 ;

    public static int CW_LEFT=800;
    public static int CW_TOP=400;
    public static int CW_RIGHT = 1380;
    public static int CW_BOTTOM= 700;


    public static int RW_LEFT = 1460;
    public static int RW_TOP = 400;
    public static int RW_RIGHT = 1810 ;
    public static int RW_BOTTOM = 900;

    public static int MIN_HUE = 0, MAX_HUE = 255;
    public static int MIN_SAT = 0, MAX_SAT = 255;
    public static int MIN_VALUE = 0, MAX_VALUE = 255;

    public static double leftH = 0.0;

    public double[] leftCb = new double[3], centerCb = new double[3], rightCb = new double[3];
    public double[] leftHSValues = new double[3], centerHSValues = new double[3], rightHSValues = new double[3];
    public double[] leftHSVBinary = new double[3], centerHSVBinary = new double[3], rightHSVBinary = new double[3];

    @java.lang.Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @java.lang.Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Mat workingHSVMat = new Mat();
        Mat workingYCrCbMat = new Mat();
        //Mat workingHSVBinaryMat = new Mat();

        Imgproc.cvtColor(frame, workingHSVMat, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(frame, workingYCrCbMat, Imgproc.COLOR_RGB2YCrCb);

        // create binary image - see https://docs.wpilib.org/en/stable/docs/software/vision-processing/wpilibpi/image-thresholding.html
        //workingHSVBinaryMat = Imgproc.inRange()

        leftWindow = new Rect(LW_LEFT, LW_TOP, LW_RIGHT- LW_LEFT, LW_BOTTOM - LW_TOP);
        trimRect(workingHSVMat, leftWindow);
        centerWindow = new Rect(CW_LEFT, CW_TOP, CW_RIGHT- CW_LEFT, CW_BOTTOM - CW_TOP);
        trimRect(workingHSVMat, centerWindow);
        rightWindow = new Rect(RW_LEFT, RW_TOP, RW_RIGHT- RW_LEFT, RW_BOTTOM - RW_TOP);
        trimRect(workingHSVMat, rightWindow);

        Mat leftHSVCrop = workingHSVMat.submat(leftWindow);
        Mat centerHSVCrop = workingHSVMat.submat(centerWindow);
        Mat rightHSVCrop = workingHSVMat.submat(rightWindow);

        // these values don't get back to parent object.
        populateValues(Core.mean(leftHSVCrop).val, leftHSValues);
        populateValues(Core.mean(centerHSVCrop).val,centerHSValues);
        populateValues(Core.mean(rightHSVCrop).val,rightHSValues);

        leftHSVCrop.release();
        centerHSVCrop.release();
        rightHSVCrop.release();
        workingHSVMat.release();

        Mat leftCbCrop = workingYCrCbMat.submat(leftWindow);
        Mat centerCbCrop = workingYCrCbMat.submat(centerWindow);
        Mat rightCbCrop = workingYCrCbMat.submat(rightWindow);

        populateValues(Core.mean(leftCbCrop).val, leftCb);
        populateValues(Core.mean(centerCbCrop).val, centerCb);
        populateValues(Core.mean(rightCbCrop).val, rightCb);

        leftCbCrop.release();
        centerCbCrop.release();
        rightCbCrop.release();
        workingYCrCbMat.release();

        return frame;

    }

    @java.lang.Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint leftPaint = new Paint();

        leftPaint.setColor(Color.GREEN);
        leftPaint.setStyle(Paint.Style.STROKE);
        leftPaint.setStrokeWidth(scaleCanvasDensity * 4);
        canvas.drawRect(makeGraphicsRect(leftWindow, scaleBmpPxToCanvasPx), leftPaint);

        Paint centerPaint = new Paint();

        centerPaint.setColor(Color.GREEN);
        centerPaint.setStyle(Paint.Style.STROKE);
        centerPaint.setStrokeWidth(scaleCanvasDensity * 4);
        canvas.drawRect(makeGraphicsRect(centerWindow, scaleBmpPxToCanvasPx), centerPaint);

        Paint rightPaint = new Paint();

        rightPaint.setColor(Color.GREEN);
        rightPaint.setStyle(Paint.Style.STROKE);
        rightPaint.setStrokeWidth(scaleCanvasDensity * 4);
        canvas.drawRect(makeGraphicsRect(rightWindow, scaleBmpPxToCanvasPx), rightPaint);
    }

    public void trimRect(Mat dest, Rect rect) {
        if (rect.x >= dest.width()) {
            rect.x = dest.width() - 2;
            rect.width = 1;
        }
        if (rect.y >= dest.height()) {
            rect.y = dest.height() - 2;
            rect.height = 1;
        }

        if (rect.x + rect.width >= dest.width()) {
            rect.width = dest.width() - rect.x - 1;
        }

        if (rect.y + rect.height >= dest.height()) {
            rect.height = dest.height() - rect.y - 1;
        }
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = Math.round(left + rect.width * scaleBmpPxToCanvasPx);
        int bottom = Math.round(top + rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    private void populateValues(double[] source, double[] dest) {
        for (int i = 0; i < 3; i++) {
            dest[i] = source[i];
        }
    }



}
