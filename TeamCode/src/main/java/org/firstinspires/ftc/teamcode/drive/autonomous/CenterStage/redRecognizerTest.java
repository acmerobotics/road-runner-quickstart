package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.CenterStage;/*
package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.CenterStage;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage.redRecognizer;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
@Disabled
@Autonomous
@Config
public class redRecognizerTest extends LinearOpMode {
    OpenCvWebcam webcam1 = null;

    redRecognizer ourCam = new redRecognizer();

    @Override
    public void runOpMode() throws InterruptedException {

        WebcamName adjustCameraName = hardwareMap.get(WebcamName.class,"Webcam 1");

        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(adjustCameraName);

        webcam1.setPipeline(ourCam);

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(1280,720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        while (!isStarted()) {
            telemetry.addData("Location", ourCam.getPixelLocationRed().name());
            telemetry.update();
        }
    }
}
*/
