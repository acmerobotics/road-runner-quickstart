package org.firstinspires.ftc.teamcode.Offseason_Code_Natalia.blahblah;
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

public class PipelineFromScratch extends OpenCvPipeline implements VisionProcessor {

//        public PipelineFromScratch.Location location = PipelineFromScratch.location;

         int cameraWidth = 920;
         int cameraHeight = 1080;
        private OpenCvCamera controlhubcamera;

        boolean isred= false;




        final Point BOTTOMRIGHT = new Point(0,0);
        final Point BOTTOMLEFT = new Point(1919,0);
        final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(0, 1079);


        Point region1_pointA = new Point(
                BOTTOMRIGHT.x,
                BOTTOMRIGHT.y);


        Point region1_pointB = new Point(
                BOTTOMLEFT.x,
                BOTTOMLEFT.y);




        Mat largerect;
        Mat HSvMat = new Mat( );

        final double RegionWidth = cameraWidth;
        final double RegionHeight = cameraHeight;


        public void init(int width, int height, CameraCalibration calibration) {
            //submats


            largerect = HSvMat.submat(new Rect(BOTTOMLEFT, BOTTOMRIGHT));
        }

//        @Override
        public Mat processFrame(Mat frame) {
            //Imgproc;
            double LowerHueThreshold = 0;
            double UpperHueThreshold = 131;
            double[] avg1;

            Scalar RedHueThreshold ;

            avg1 = new double[]{Core.mean(largerect).val[0]};

            if(avg1[0] < UpperHueThreshold && avg1[0] > LowerHueThreshold){
                isred = true;


            }
            return null;





        }
        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            // Not useful either
        }

//        @Override
//        public void onViewportTapped() {
//            //screen is clicked or tapped
//
//        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
        return null;

    }


    }





