package org.firstinspires.ftc.teamcode;


import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@Autonomous(name = "#Auto")

public class Auto2 extends LinearOpMode {

    private DcMotorEx lift, leftRotate, rightRotate;
    private Servo rotate, left, right;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera


    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    double linearVelX = 0;
    double linearVelY = 0;
    double angularVel = 0;

    @Override


    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        TwoDeadWheelLocalizer twoDeadWheelLocalizer = new TwoDeadWheelLocalizer(hardwareMap, drive.lazyImu.get(), 1870);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        initAprilTag();

        double tagBearing = 0;
        double tagRange = 0;
        double tagYaw = 0;
        double tagY = 0;
        double tagX = 0;
        double lastHeading = 0;
        double startX = drive.pose.position.x;
        double startY = drive.pose.position.y;
        double targetX;
        double targetY;
        waitForStart();
        resetRuntime();
        if (opModeIsActive()) {

            while (opModeIsActive()) { //Main loop

                telemetryAprilTag();

                if (getRuntime() <5) {
                    driveToPosition(100, 0, drive, twoDeadWheelLocalizer);
                }

                //Lift
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                if (!gamepad1.b) {
                    for (AprilTagDetection detection : currentDetections) {
                        if (detection.id == 15 || detection.id == 11) {
                            tagBearing = detection.ftcPose.bearing;
                            tagRange = detection.ftcPose.range;
                        }
                    }

                }
                if (currentDetections.isEmpty()){
                    tagBearing = 0;
                    tagRange = 0;
                }
                //double absoluteY = (Math.cos((drive.pose.heading.toDouble() - 90) + tagBearing)*tagRange);

                if (getRuntime() <= 3.00) {
                    linearVelX = 1;
                }
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(linearVelX, linearVelY), angularVel)); //Final Drive Inputs
            }
        }
    }

    private void driveToPosition(double abX, double abY, MecanumDrive drive, TwoDeadWheelLocalizer twoDeadWheelLocalizer) {
        double positionX = twoDeadWheelLocalizer.par.getPositionAndVelocity().position;
        double positionY = twoDeadWheelLocalizer.perp.getPositionAndVelocity().position;

        while (positionX != abX || positionY != abY) {
            positionX = twoDeadWheelLocalizer.par.getPositionAndVelocity().position;
            positionY = twoDeadWheelLocalizer.perp.getPositionAndVelocity().position;
            double powerX = ((abX - positionX)*-1)/1000;
            double powerY = ((abY - positionY)*-1)/1000;
            linearVelX = powerX;
            linearVelY = powerY;
            runTelemetry(drive);
            /*if (drive.pose.position.x < abX) {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(1, 0), 0));
                runTelemetry(drive);
            }*/

        }

    }
    private void runTelemetry(MecanumDrive drive){
        telemetry.addLine(String.format("%6.1f time", getRuntime()));
        telemetry.addLine(String.format("%6.1f Diffx", ((100 - drive.pose.position.x)*-1)/1000));
        //telemetry.addLine(String.format("%6.1f Pose X", drive.pose.position.x));
        telemetry.addLine(String.format("%6.1f Pose X", drive.pose.position.x));
        telemetry.addLine(String.format("%6.1f Pose Y", drive.pose.position.y));
        telemetry.addLine(String.format("%6.1f heading (deg)", Math.toDegrees(drive.pose.heading.toDouble())));
        telemetryAprilTag();
        telemetry.update();
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

}
