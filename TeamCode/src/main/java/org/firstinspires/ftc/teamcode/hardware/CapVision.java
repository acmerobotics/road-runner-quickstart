package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.common.reflection.qual.GetConstructor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.teamcode.hardware.util.kellen;
@Config
public class CapVision extends Mechanism {
    //camera object
    private OpenCvWebcam webcam;
    public static boolean preview = true;
    //pipeline
    public static String color = "green";
    private kellen regions = new kellen(color);
    int viewid;

    @Override
    public void init(HardwareMap hwMap) {
        viewid = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        if(preview) webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "camera"),viewid);
        else webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "camera"));

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                //webcam.startStreaming(320, 180, OpenCvCameraRotation.UPRIGHT);
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

                FtcDashboard.getInstance().startCameraStream(webcam, 0);
                setPipeline();

            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

    }

    //set pipeline within the linear opmode
    public void setPipeline(){
        webcam.setPipeline(regions);
    }
    public int whichRegion() {
        return regions.whichRegion();
    }

}
