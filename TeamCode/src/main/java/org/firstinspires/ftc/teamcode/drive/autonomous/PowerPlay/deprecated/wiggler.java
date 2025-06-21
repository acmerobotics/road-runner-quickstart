package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.PowerPlay.deprecated;/*
package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.PowerPlay.deprecated;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.visionPowerPlay.gonkaPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Disabled
@Config
@Autonomous(group = "testing")

public class wiggler extends LinearOpMode {

    OpenCvWebcam camera = null;
    gonkaPipeline wiggleDetector = new gonkaPipeline();
    gonkaPipeline.wiggleDirection moveDir;
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        Trajectory moveLeft = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(2)
                .build();

        Trajectory moveRight = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(2)
                .build();

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "camera");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        camera.setPipeline(wiggleDetector);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();

        while (true) {

            moveDir = wiggleDetector.getDirection();
            if (moveDir == gonkaPipeline.wiggleDirection.LEFT) {
                telemetry.addData("Move left", null);
                drive.followTrajectory(moveLeft);
            } else if (moveDir == gonkaPipeline.wiggleDirection.RIGHT) {
                telemetry.addData("Move right", null);
                drive.followTrajectory(moveRight);
            } else if (moveDir == gonkaPipeline.wiggleDirection.STOP) {
                telemetry.addData("Aligned", null);
            } else {
                telemetry.addData("IDK", null);
            }
            telemetry.update();
            if (isStopRequested()) {
                camera.stopStreaming();
                break;
            }
        }
    }
}
*/
