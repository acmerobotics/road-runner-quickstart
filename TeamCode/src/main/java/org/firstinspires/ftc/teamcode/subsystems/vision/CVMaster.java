package org.firstinspires.ftc.teamcode.subsystems.vision;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.SortOrder;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.util.misc.FullPose2d;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

@Config
public class CVMaster {
    public Limelight3A limelight;
    EnumMap<LLPipeline, Integer> llPipelines = new EnumMap<>(LLPipeline.class);

    VisionPortal portal;
    ColorBlobLocatorProcessor redProcessor;
    ColorBlobLocatorProcessor blueProcessor;
    ColorBlobLocatorProcessor yellowProcessor;
    EOCVPipeline activeCV = EOCVPipeline.YELLOW_SAMPLE;

    List<ColorBlobLocatorProcessor.Blob> rawPotTargets;
    List<Pose3D> targets = new ArrayList<>();
    Pose2d poseAtSnapshot;

    public final float WEBCAM_X_OFFSET = 0;
    public final float WEBCAM_Y_OFFSET = 0;
    public final float WEBCAM_Z_OFFSET = 0;
    public final float WEBCAM_PITCH_OFFSET = 0;
    public final float WEBCAM_VFOV = 0;
    public final float WEBCAM_HFOV = 0;
    public final double WEBCAM_H = 480;
    public final double WEBCAM_W = 640;

    public CVMaster(Limelight3A ll3a, WebcamName camera) {
        limelight = ll3a;

        llPipelines.put(LLPipeline.APRILTAGS, 0);
        llPipelines.put(LLPipeline.YELLOW_SAMPLE, 1);
        llPipelines.put(LLPipeline.RED_SAMPLE, 2);
        llPipelines.put(LLPipeline.BLUE_SAMPLE, 3);
//        pipelines.put(Pipeline.RED_YELLOW_SAMPLE, 4);
//        pipelines.put(Pipeline.BLUE_YELLOW_SAMPLE, 5);

        redProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1,1,1,-1))
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        blueProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1,1,1,-1))
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        yellowProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1,1,1,-1))
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(camera)
                .setCameraResolution(new Size(1280, 800))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessors(redProcessor, yellowProcessor, blueProcessor)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
        setEOCVPipeline(EOCVPipeline.YELLOW_SAMPLE);
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
        portal.close();
    }

    public void kill() {
        limelight.stop();
        limelight.shutdown();
        portal.stopLiveView();
        portal.stopStreaming();
        portal.close();
    }

    public void setLLPipeline(LLPipeline p) {
        int pID = llPipelines.get(p);
        limelight.pipelineSwitch(pID);
    }

    public void setEOCVPipeline(EOCVPipeline pipeline) {
        if (pipeline == EOCVPipeline.RED_SAMPLE) {
            portal.setProcessorEnabled(redProcessor, true);
            portal.setProcessorEnabled(blueProcessor, false);
            portal.setProcessorEnabled(yellowProcessor, false);
//            setDashboardStream(redProcessor);
        } else if (pipeline == EOCVPipeline.BLUE_SAMPLE) {
            portal.setProcessorEnabled(redProcessor, false);
            portal.setProcessorEnabled(blueProcessor, true);
            portal.setProcessorEnabled(yellowProcessor, false);
//            setDashboardStream(blueProcessor);
        } else if (pipeline == EOCVPipeline.YELLOW_SAMPLE) {
            portal.setProcessorEnabled(redProcessor, false);
            portal.setProcessorEnabled(blueProcessor, false);
            portal.setProcessorEnabled(yellowProcessor, true);
//            setDashboardStream(yellowProcessor);
        }
        activeCV = pipeline;
    }

    /**
    Relocalize using Limelight MegaTag2. <b>Requires current robot heading in order to accurately work</b>
     @return Current robot pose if tag in sight, or null if tag not in sight
     @param heading current accurate robot heading
     */
    public Pose2d mt2Relocalize(double heading) {
        limelight.updateRobotOrientation(Math.toDegrees(heading));
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D pose = result.getBotpose_MT2();
            return new Pose2d(new Vector2d((pose.getPosition().y*39.3701007874), (pose.getPosition().x*9.3701007874)), pose.getOrientation().getYaw(AngleUnit.RADIANS));
//            return new Pose2d(pose.getPosition().x,
//                    pose.getPosition().y,
//                    Rotation2d.fromDegrees(pose.getOrientation().getPitch(AngleUnit.DEGREES)));
        }

        return null;
    }

    public LLResult mt2RelocalizeRAW(double heading) {
        limelight.updateRobotOrientation(Math.toDegrees(heading));
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            return result;
        }

        return null;
    }

    /**
     Relocalize using Limelight MegaTag1. <b>LESS STABLE THAN MEGATAG2, ONLY USE IF HEADING IS STALE</b>
     @return Current robot pose if tag in sight, or null if tag not in sight
     */
    public Pose2d mt1Relocalize() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D pose = result.getBotpose();
            return new Pose2d(pose.getPosition().x,
                    pose.getPosition().y,
                    pose.getOrientation().getPitch(AngleUnit.RADIANS));
        }

        return null;
    }

    public void updatePotentialTargetList(EOCVPipeline color, Pose2d robotPose) {
        if (activeCV != color) {
            setEOCVPipeline(color);
        }


        List<ColorBlobLocatorProcessor.Blob> rawTargets;

        if (activeCV == EOCVPipeline.RED_SAMPLE) {
            rawTargets = redProcessor.getBlobs();
        } else if (activeCV == EOCVPipeline.BLUE_SAMPLE) {
            rawTargets = blueProcessor.getBlobs();
        } else {
            rawTargets = yellowProcessor.getBlobs();
        }

        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, rawTargets);
        ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, rawTargets);

        targets = new ArrayList<>();
        for (int i = 0; i < Math.min(5, rawTargets.size()); i++) {
            targets.add(calculateBlobFieldCoordinates(rawTargets.get(i)));
        }

        rawPotTargets = rawTargets;
        poseAtSnapshot = robotPose;
    }

    public Pose3D calculateBlobFieldCoordinates(ColorBlobLocatorProcessor.Blob blob) {
        // Find pixel coordinates of the target
        double x = blob.getBoxFit().center.x;
        double y = blob.getBoxFit().center.y;

        // convert from image coords to normalized coords
        double u = ((2*x)/WEBCAM_W) - 1;
        double v = ((2*y)/WEBCAM_H) - 1;

        // find angle offsets
        double phi_x = u * (WEBCAM_HFOV / 2);
        double phi_y = v * (WEBCAM_VFOV / 2);

        // Calculate world coords relative to the camera (assumes camera is (0,0) )
        double world_x = (WEBCAM_Z_OFFSET * Math.tan(phi_x)) / ((Math.sin(WEBCAM_PITCH_OFFSET) * Math.tan(phi_y)) - Math.cos(WEBCAM_PITCH_OFFSET));
        double world_y = (WEBCAM_Z_OFFSET * ((Math.cos(WEBCAM_PITCH_OFFSET) * Math.tan(phi_y)) + Math.sin(WEBCAM_PITCH_OFFSET))) / ((Math.sin(WEBCAM_PITCH_OFFSET) * Math.tan(phi_y)) - Math.cos(WEBCAM_PITCH_OFFSET));

        // Calculate the coords of the camera on the field using the offsets
        double cam_x = poseAtSnapshot.position.x + (WEBCAM_X_OFFSET * Math.cos(poseAtSnapshot.heading.toDouble())) - (WEBCAM_Y_OFFSET * Math.sin(poseAtSnapshot.heading.toDouble()));
        double cam_y = poseAtSnapshot.position.y + (WEBCAM_X_OFFSET * Math.sin(poseAtSnapshot.heading.toDouble())) + (WEBCAM_Y_OFFSET * Math.cos(poseAtSnapshot.heading.toDouble()));

        // Finally calculate the field coords of the target
        double field_x = cam_x + (world_x * Math.cos(poseAtSnapshot.heading.toDouble())) - (cam_y * Math.sin(poseAtSnapshot.heading.toDouble()));
        double field_y = cam_y + (world_x * Math.sin(poseAtSnapshot.heading.toDouble())) + (world_y * Math.cos(poseAtSnapshot.heading.toDouble()));

        return new Pose3D(
                new Position(DistanceUnit.INCH, field_x, field_y, 0, System.currentTimeMillis()),
                new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, System.currentTimeMillis()));
    }

    public Pose3D findOptimalTarget(Pose2d robotPose) {
        Pose3D t = targets.get(0);
        targets.remove(0);
        return t;
    }

    public FullPose2d calculateRobotFullPose(Pose3D target, double constX, double constY) {
        double a = Math.abs((target.getPosition().y - constY));
        double b = Math.abs((target.getPosition().x - constX));
        double c = Math.sqrt((Math.pow(a, 2) + Math.pow(b, 2)));
        double heading = Math.acos((a/c));
        if ((target.getPosition().y - constY) < 0 || (target.getPosition().x - constX) < 0) {
            heading = -1 * heading;
        }

        return new FullPose2d(constX, constY, heading, c);
    }

    public Pose2d llGetClosestYellowLocation() {
        setLLPipeline(LLPipeline.YELLOW_SAMPLE);

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            return new Pose2d(result.getTx(), result.getTy(), result.getTa());
        }
        return null;
    }

    public Pose2d llGetClosestRedLocation() {
        setLLPipeline(LLPipeline.RED_SAMPLE);

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            return new Pose2d(result.getTx(), result.getTy(), result.getTa());
        }
        return null;
    }

    public Pose2d llGetClosestBlueLocation() {
        setLLPipeline(LLPipeline.BLUE_SAMPLE);

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            return new Pose2d(result.getTx(), result.getTy(), result.getTa());
        }
        return null;
    }

    public enum LLPipeline {
        APRILTAGS,
        RED_SAMPLE,
        BLUE_SAMPLE,
        YELLOW_SAMPLE,
        RED_YELLOW_SAMPLE,
        BLUE_YELLOW_SAMPLE,
        RESERVED
    }

    public enum EOCVPipeline {
        RED_SAMPLE,
        BLUE_SAMPLE,
        YELLOW_SAMPLE,
    }

    public void setDashboardStream(ColorBlobLocatorProcessor pcr) {
        FtcDashboard.getInstance().startCameraStream((CameraStreamSource) pcr, 0);
    }
}
