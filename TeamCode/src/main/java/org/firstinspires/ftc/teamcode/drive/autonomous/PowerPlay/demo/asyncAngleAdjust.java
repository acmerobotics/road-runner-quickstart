package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.PowerPlay.demo;/*
package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.PowerPlay.demo;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.teamcode.drive.opmode.visionPowerPlay.poleFinder;
@Disabled
@Config
@Autonomous(group = "demo")
public class asyncAngleAdjust extends LinearOpMode {
    SampleMecanumDrive drive;
    OpenCvWebcam camera = null;
    poleFinder poleFinderPipeline = new poleFinder();
    poleFinder.poleLocation moveDir;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "adjustCamera");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        camera.setPipeline(poleFinderPipeline);

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

        adjustAngle(drive);

        while (!isStopRequested()) {}

    }

    public void adjustAngle(SampleMecanumDrive _drive) {
        double maxAngVel = Math.toRadians(20.0);
        double maxAngAccel = Math.toRadians(10.0);
        moveDir = poleFinderPipeline.getLocation();
        poleFinder.poleLocation prevDir = moveDir;

        TrajectorySequence turnLeft = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(45), maxAngVel, maxAngAccel)
                .build();

        TrajectorySequence turnRight = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(-45), maxAngVel, maxAngAccel)
                .build();

        TrajectorySequence doNothing = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(0), maxAngVel, maxAngAccel)
                .build();

        if (moveDir == poleFinder.poleLocation.ALIGNED) {
            return;
        } else if (moveDir == poleFinder.poleLocation.LEFT) {
            _drive.followTrajectorySequenceAsync(turnLeft);
        } else if (moveDir == poleFinder.poleLocation.RIGHT) {
            _drive.followTrajectorySequenceAsync(turnRight);
        }

        while (moveDir == poleFinder.poleLocation.RIGHT) {
            _drive.update();
            moveDir = poleFinderPipeline.getLocation();
        }

        _drive.followTrajectorySequenceAsync(doNothing);
        _drive.update();

    }
}
*/
