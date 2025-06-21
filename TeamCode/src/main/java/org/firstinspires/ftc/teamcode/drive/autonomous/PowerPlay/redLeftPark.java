package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.PowerPlay; /**package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.PowerPlay;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.visionPowerPlay.parkingZoneFinder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Disabled
@Autonomous(name = "Red Left Park", group = "competition")
public class redLeftPark extends LinearOpMode {
    private final Pose2d startPose = new Pose2d(36, -64.25, Math.toRadians(90)); // our Starting pose allows us to know our postions of the robot and know what way it os looking at

    private final Pose2d scorePose = new Pose2d(40, -11, Math.toRadians(90));

    private final Pose2d[] parkingSpots = {new Pose2d(12, -17, Math.toRadians(90)), new Pose2d(36,
            -20, Math.toRadians(90)), new Pose2d(64, -15, Math.toRadians(90))};

    private final int width = 1280, height = 720;
    private final double travelSpeed = 45.0, travelAccel = 30.0;

    SampleMecanumDrive drive;

    OpenCvWebcam adjustCamera = null;
    parkingZoneFinder parkingZonePipeline = new parkingZoneFinder();
    parkingZoneFinder.parkingZone zone;

    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);
        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(scorePose,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                .build();
        WebcamName adjustCameraName = hardwareMap.get(WebcamName.class, "adjustCamera");
        adjustCamera = OpenCvCameraFactory.getInstance().createWebcam(adjustCameraName);

        // Set the camera's pipeline
        adjustCamera.setPipeline(parkingZonePipeline);

        // Open the camera
        adjustCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                adjustCamera.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        while (!isStarted()) {
            zone = parkingZonePipeline.getParkingZone();
            telemetry.addData("Parking Zone", zone);
            telemetry.update();
        }
        adjustCamera.stopStreaming();
        adjustCamera.closeCameraDevice();

        drive.followTrajectorySequence(goToStack);
        if (zone == parkingZoneFinder.parkingZone.ZONE1) { parkBot(drive, 0, parkingSpots); }
        else if (zone == parkingZoneFinder.parkingZone.ZONE2) { parkBot(drive, 1, parkingSpots); }
        else if (zone == parkingZoneFinder.parkingZone.ZONE3) { parkBot(drive, 2, parkingSpots); }
        else { parkBot(drive, 1, parkingSpots); }
    }

    private void parkBot(SampleMecanumDrive _drive, int _zone, Pose2d[] locations) {
        _drive.updatePoseEstimate();
        Trajectory moveToPark = _drive.trajectoryBuilder(_drive.getPoseEstimate())
                .lineToLinearHeading(locations[_zone])
                .build();

        _drive.setGrip(false);
        _drive.setExtension(50);
        _drive.setHeight(4400);

        _drive.followTrajectory(moveToPark);

        _drive.setHeight(100);
    }

}
*/