package org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Camerainitialization {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // You're gonna have to check Vuforia for the details once hardware is installed :/ Also wtf happened to hardwareMap?
        // Use camera.pauseViewport(); to turn live viewport on and off.
        // ONLY TURN LIVE VIEWPORT ON FOR DEBUG. UNDER ABSOLUTELY NO CIRCUMSTANCES SHOULD IT BE ON DURING COMPETITION!!!!
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "NAME_OF_CAMERA_IN_CONFIG_FILE");
        //Need to come back to this and get exact names
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        public void openCameraDeviceAsync (OpenCvCamera.AsyncCameraOpenListener cameraOpenListener) {}

        public void onOpened() {
        camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
        // gotta input camera resolution once we get that info
        // Usually this is where you'll want to start streaming from the camera
    }

        public void onError () {}
}
