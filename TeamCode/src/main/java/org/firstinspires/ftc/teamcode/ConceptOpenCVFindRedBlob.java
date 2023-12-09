package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
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

    WebcamName webcamName= hardwareMap.get(WebcamName.class, "Webcam 1");
    VisionPortal visionPortal;
    OpenCVProcessor visionProcessor;
    final int RESOLUTION_WIDTH= 640;
    final int RESOLUTION_HEIGHT=480;

    //Rect uses left x, top y, width, height in camera coodinates
    final Rect LEFT_WINDOW =new Rect(2,2,316,476);
    final Rect RIGHT_WINDOW = new Rect(322, 2, 316, 476);

    public void runOpMode() {
        visionProcessor = new OpenCVProcessor();
        visionPortal = new VisionPortal.Builder()
			    .setCamera(webcamName)
                .addProcessors((VisionProcessor) visionProcessor)
                .setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
                .build();
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Red blob on side: ", visionProcessor.getSide());
        }
    }



    class OpenCVProcessor implements VisionProcessor {

        public final int BLUE_CHANNEL = 1;
        public final int RED_CHANNEL = 2;
        public String side = "None";  			// accessible side ID
        public float leftMean, rightMean; 		// accessible mean red metrics
        public final int THRESHOLD = 5;   // sets sensitivity of detector.  If the mean red

        public String getSide() {
        }
        }

        @java.lang.Override
        public void init(int width, int height, CameraCalibration calibration) {

        }

        @java.lang.Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            return null;
        }

        @java.lang.Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        }
        // of left is more than THRESHOLD than
        // mean red of right, then left is the side.

        @interface Override
        public void init() {
        }

        public Object processFrame(Mat frame, long captureTimeNanos) {
            if (frame == null) {
                return null;
            }
            Mat workingMat = new Mat();
            Imgproc.cvtColor(frame, workingMat, Imgproc.RGB2YCrCb);
            Mat leftCrop = workingMat.submat(LEFT_WINDOW);
            Core.extractChannel(leftCrop, leftCrop, RED_CHANNEL);

            rightCrop = workingMat.submat(RIGHT_WINDOW);
            Core.extractChannel(rightCrop, rightCrop, RED_CHANNEL);

            leftAvg = Core.mean(leftCrop).val[0];
            rightAvg = Core.mean(rightCrop).val[0];

            if (leftAvg - rightAvg > THRESHOLD) {
                side = “Left”;
            }
            else if (rightAvg - leftAvg > THRESHOLD) {
                side = “Right”;
            }
            else {side = “None”;}

// release all created matrices!  Otherwise a memory leak will occur.
            workingMat.release();
            leftCrop.release();
            rightCrop.release();
            return frame;
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



        @Override
        public void onDrawFrame(Canvas canvas,
                                int onscreenWidth,
                                int onscreenHeight,
                                float scaleBmpPxToCanvasPx,
                                float scaleCanvasDensity, Object userContext) {
            paint leftPaint = new Paint();
            if side.equals(“Left”) {
                leftPaint.setColor(Color.RED);
            }
	else {
                leftPaint.setColor(Color.GREEN);
            }
            leftPaint.setStyle(Paint.Style.STROKE); 38 leftPaint.setStrokeWidth(scaleCanvasDensity * 4);

            canvas.drawRect(makeGraphicsRect(LEFT_WINDOW, scaleBmpPxToCanvasPx), leftPaint);

            paint rightPaint = new Paint();
            if side.equals(“Right”) {
                rightPaint.setColor(Color.RED);
            }
	else {
                rightPaint.setColor(Color.GREEN);
            }
            rightPaint.setStyle(Paint.Style.STROKE); 38 rightPaint.setStrokeWidth(scaleCanvasDensity * 4);

            canvas.drawRect(makeGraphicsRect(RIGHT_WINDOW, scaleBmpPxToCanvasPx), rightPaint);
        }
    }



