package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.visionPiplines.DetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class ConeDetectionSubsystem extends Subsystem {

	OpenCvPipeline pipeline = new DetectionPipeline();
	OpenCvWebcam webcam;

	Dashboard dash;

	public ConeDetectionSubsystem(Dashboard dashboard) {
		this.dash = dashboard;
	}


	@Override
	public void initAuto(HardwareMap hwMap) {
		int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
		webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


		webcam.setPipeline(pipeline);

		webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
		{
			@Override
			public void onOpened()
			{

				webcam.startStreaming(640, 480 , OpenCvCameraRotation.SIDEWAYS_RIGHT);
			}

			@Override
			public void onError(int errorCode) {

			}
		});

		dash.startCameraStream(webcam,30);

	}

	@Override
	public void periodic() {

	}

	@Override
	public void shutdown() {
		webcam.closeCameraDevice();
	}
}
