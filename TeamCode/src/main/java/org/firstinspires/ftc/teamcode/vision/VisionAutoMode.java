package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "VisionAuto", group = "Auto")
public class VisionAutoMode extends LinearOpMode {
    OpenCvCamera phoneCamera;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        phoneCamera = OpenCvCameraFactory.getInstance()
                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        final Vision detector = new Vision(telemetry);
        phoneCamera.setPipeline(detector);
        phoneCamera.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        phoneCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                    }

                    @Override
                    public void onError(int errorCode) {
                        throw new RuntimeException(String.valueOf(errorCode));
                    }
                }
        );

        waitForStart();
        switch (detector.getSleeveDirection()) {
            case LEFT:

                break;
            case RIGHT:
                // ...
                break;
            case CENTER:
                // ...
        }
        phoneCamera.stopStreaming();
    }
}
