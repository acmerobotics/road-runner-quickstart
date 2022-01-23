//All this code was heavily aided by this YouTube guide: https://www.youtube.com/watch?v=JO7dqzJi8lw; check it out if you have further questions or difficulty
package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.CVPipelines.RingCounter;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

//A new class will probably be created for a normal webcam.
//This code works if you are using a phone as a robot controller, and want to use it as your camera.
public class PhoneCamera extends Mechanism{
    //here I initialize the phone camera and the pipeline; there are other ways to do this if you prefer
    private OpenCvInternalCamera phoneCam;
    private RingCounter counter = new RingCounter();
    private boolean flash;

    //this basically links variable phoneCam to the actual phone camera; after that, it links phoneCam to the pipeline, counter; I don't know if "link" is the correct term
    public void init(HardwareMap hwMap){
        int cameraMonitorViewID = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK,cameraMonitorViewID);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(counter);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }

    //It toggles the flashlight... OpenCV has some other features utilizing OpenCvInternalCamera
    public void flash(){
        phoneCam.setFlashlightEnabled(!flash);
        flash = !flash;
    }
    //Returns the percentage of the target color in the target region; go into RingCounter to learn more
    public double regionValue(){
        return counter.regionValue();
    }
}
