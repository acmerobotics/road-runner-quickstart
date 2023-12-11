package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

//TODO: test
@Config
@Autonomous(name = "ConceptOpenCVFindRedBlob", group= "Concept")
public class ConceptOpenCVFindRedBlob extends LinearOpMode {

    WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
    VisionPortal visionPortal;
    OpenCVProcessor visionProcessor;
    final static int RESOLUTION_WIDTH = 640;
    final static int RESOLUTION_HEIGHT = 480;

    public static int LW_LEFT = 2;
    public static int LW_TOP = 2;
    public static int LW_WIDTH = RESOLUTION_WIDTH / 2 - 4;
    public static int LW_HEIGHT = RESOLUTION_HEIGHT - 2;

    public static int RW_LEFT = RESOLUTION_WIDTH / 2 + 2;
    public static int RW_TOP = 2;
    public static int RW_WIDTH = RESOLUTION_WIDTH / 2 - 4;
    public static int RW_HEIGHT = RESOLUTION_HEIGHT - 2;

    public static double THRESHOLD = 5.0;


    //Rect uses left x, top y, width, height in camera coodinates
    Rect leftWindow, rightWindow;

    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        visionProcessor = new OpenCVProcessor();
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcamName)
                .addProcessors((VisionProcessor) visionProcessor)
                .setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
                .build();
        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Hit Y to see input frame, A to see output frame");
            telemetry.addData("Red blob on side: ", visionProcessor.getSide());
            telemetry.addData("mean red on left side: ", visionProcessor.leftMean);
            telemetry.addData("mean red on right side: ", visionProcessor.rightMean);
            telemetry.addData("THRESHOLD: ", THRESHOLD);
            if (gamepad1.y) {
                visionProcessor.setView(visionProcessor.INPUT);
            }
            else if (gamepad1.a) {
                visionProcessor.setView(visionProcessor.OUTPUT);
            }
        }
    }


    class OpenCVProcessor implements VisionProcessor {

        public final int BLUE_CHANNEL = 1;
        public final int RED_CHANNEL = 2;
        public String side = "None";            // accessible side ID
        public double leftMean, rightMean;        // accessible mean red metrics
        public final int THRESHOLD = 5;   // sets sensitivity of detector.  
        // if the mean red of one side is greater than THRESHOLD + mean of the other, that side is "Red"

        public Boolean INPUT = true;
        public Boolean OUTPUT = false;
        Boolean inOut = INPUT;
        public String getSide() {
            return side;
        }

        public void setView(Boolean inOut) {
            this.inOut = inOut;
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
            leftWindow = new Rect(LW_LEFT, LW_TOP, LW_WIDTH, LW_HEIGHT);
            rightWindow = new Rect(RW_LEFT, RW_TOP, RW_WIDTH, RW_HEIGHT);
            Imgproc.cvtColor(frame, workingMat, Imgproc.COLOR_BGR2YCrCb);   // Possibly: Use HSV.  Use inRange() to convert to binary matrix.
            Mat leftCrop = workingMat.submat(leftWindow);
            Core.extractChannel(leftCrop, leftCrop, RED_CHANNEL);

            Mat rightCrop = workingMat.submat(rightWindow);
            Core.extractChannel(rightCrop, rightCrop, RED_CHANNEL);

            leftMean = Core.mean(leftCrop).val[0];
            rightMean = Core.mean(rightCrop).val[0];

            if (leftMean - rightMean > THRESHOLD) {
                side = "Left";
            } else if (rightMean - leftMean > THRESHOLD) {
                side = "Right";
            } else {
                side = "None";
            }

// release all created matrices!  Otherwise a memory leak will occur.
            leftCrop.release();
            rightCrop.release();
            if (inOut) {
                workingMat.release();
                return frame;
            }
            else {
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


// convert a camera rectangle (x,y,width,height) into a screen rectangle
// (left, top, right, bottom)

        private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
            int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
            int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
            int right = Math.round(left + rect.width * scaleBmpPxToCanvasPx);
            int bottom = Math.round(top + rect.height * scaleBmpPxToCanvasPx);

            return new android.graphics.Rect(left, top, right, bottom);
        }
    }
}




