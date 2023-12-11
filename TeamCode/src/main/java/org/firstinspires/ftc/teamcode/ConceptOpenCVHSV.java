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

/* ToDO - fix communication of values.  leftValues, centerValues, rightValues stay at 0.0

 */

@Config
public class ConceptOpenCVHSV implements VisionProcessor {

    public static Boolean showOutput = false;   // for config purposes - can view output rather than input.
                                                // DOES NOT PLAY NICELY WITH OTHER VISION PROCESSORS THAT EXPECT THE INPUT STREAM

    Rect leftWindow, centerWindow, rightWindow;
    public static int LW_LEFT = 2, LW_RIGHT = 200, LW_TOP = 2, LW_BOTTOM = 400;
    public static int CW_LEFT = 202, CW_RIGHT = 400, CW_TOP = 2, CW_BOTTOM = 400;

    public static int RW_LEFT = 402, RW_RIGHT = 600, RW_TOP = 2, RW_BOTTOM = 400;

    public static double leftH = 0.0;

    public double[] leftValues = new double[3], centerValues = new double[3], rightValues = new double[3];
    @java.lang.Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @java.lang.Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Mat workingMat = new Mat();
        Imgproc.cvtColor(frame, workingMat, Imgproc.COLOR_RGB2HSV);
        leftWindow = new Rect(LW_LEFT, LW_TOP, LW_RIGHT- LW_LEFT, LW_BOTTOM - LW_TOP);
        trimRect(workingMat, leftWindow);
        centerWindow = new Rect(CW_LEFT, CW_TOP, CW_RIGHT- CW_LEFT, CW_BOTTOM - CW_TOP);
        trimRect(workingMat, centerWindow);
        rightWindow = new Rect(RW_LEFT, RW_TOP, RW_RIGHT- RW_LEFT, RW_BOTTOM - RW_TOP);
        trimRect(workingMat, rightWindow);

        Mat leftCrop = workingMat.submat(leftWindow);
        Mat centerCrop = workingMat.submat(centerWindow);
        Mat rightCrop = workingMat.submat(rightWindow);

        // these values don't get back to parent object.
        populateValues(Core.mean(leftCrop).val, leftValues);
        populateValues(Core.mean(centerCrop).val,centerValues);
        populateValues(Core.mean(rightCrop).val,rightValues);

        leftCrop.release();
        centerCrop.release();
        rightCrop.release();

        if (showOutput) {
            frame.release();
            return workingMat;
        }
        else {
            workingMat.release();
            return frame;
        }
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
