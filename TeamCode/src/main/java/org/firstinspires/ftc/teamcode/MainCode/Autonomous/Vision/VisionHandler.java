package org.firstinspires.ftc.teamcode.testing.openCvVision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class VisionHandler {
    ElementDetectionPipeline pipeline = new ElementDetectionPipeline();
    OpenCvCamera camera;
    boolean ready = false;
    void init(HardwareMap hardwareMap){

        // I have no clue why this works
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                ready = true;
            }
            public void onError(int errorCode) {}
        });
    }

    void setClose(){
        pipeline.setPositionParameters(
                VisionParameters.closeStartX,
                VisionParameters.closeStartY,
                VisionParameters.closeEndX,
                VisionParameters.closeEndY
        );
    }
    void setFar(){
        pipeline.setPositionParameters(
                VisionParameters.farStartX,
                VisionParameters.farStartY,
                VisionParameters.farEndX,
                VisionParameters.farEndY
        );
    }
    void setRed(){
        pipeline.setColorParameters(
                VisionParameters.redHueMin,
                VisionParameters.redHueMax,
                VisionParameters.redSatMin,
                VisionParameters.redSatMax,
                VisionParameters.redValMin,
                VisionParameters.redValMax
        );
    }
    void setBlue(){
        pipeline.setColorParameters(
                VisionParameters.blueHueMin,
                VisionParameters.blueHueMax,
                VisionParameters.blueSatMin,
                VisionParameters.blueSatMax,
                VisionParameters.blueValMin,
                VisionParameters.blueValMax
        );
    }
    double read() throws InterruptedException {
        camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
        Thread.sleep(VisionParameters.readTime);
        double amt = pipeline.amount;
        camera.stopStreaming();
        return amt;
    }
}
