package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.EnumMap;
import java.util.List;

public class CVMaster {
    public Limelight3A limelight;
    EnumMap<Pipeline, Integer> pipelines = new EnumMap<>(Pipeline.class);

    public CVMaster(Limelight3A ll3a) {
        limelight = ll3a;

        pipelines.put(Pipeline.APRILTAGS, 0);
        pipelines.put(Pipeline.YELLOW_SAMPLE, 1);
        pipelines.put(Pipeline.RED_SAMPLE, 2);
        pipelines.put(Pipeline.BLUE_SAMPLE, 3);
//        pipelines.put(Pipeline.RED_YELLOW_SAMPLE, 4);
//        pipelines.put(Pipeline.BLUE_YELLOW_SAMPLE, 5);
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public void kill() {
        limelight.stop();
        limelight.shutdown();
    }

    public void setPipeline(Pipeline p) {
        int pID = pipelines.get(p);
        limelight.pipelineSwitch(pID);
    }

    /**
    Relocalize using Limelight MegaTag2. <b>Requires current robot heading in order to accurately work</b>
     @return Current robot pose if tag in sight, or null if tag not in sight
     @param heading current accurate robot heading
     */
    public Pose2d mt2Relocalize(double heading) {
        limelight.updateRobotOrientation(heading);
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D pose = result.getBotpose_MT2();
            return new Pose2d(pose.getPosition().x,
                    pose.getPosition().y,
                    Rotation2d.fromDegrees(pose.getOrientation().getPitch(AngleUnit.DEGREES)));
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
                    Rotation2d.fromDegrees(pose.getOrientation().getPitch(AngleUnit.DEGREES)));
        }

        return null;
    }

    public Pose2d getClosestYellowLocation() {
        setPipeline(Pipeline.YELLOW_SAMPLE);

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            return new Pose2d(result.getTx(), result.getTy(), Rotation2d.fromDegrees(result.getTa()));
        }
        return null;
    }

    public Pose2d getClosestRedLocation() {
        setPipeline(Pipeline.RED_SAMPLE);

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            return new Pose2d(result.getTx(), result.getTy(), Rotation2d.fromDegrees(result.getTa()));
        }
        return null;
    }

    public Pose2d getClosestBlueLocation() {
        setPipeline(Pipeline.BLUE_SAMPLE);

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            return new Pose2d(result.getTx(), result.getTy(), Rotation2d.fromDegrees(result.getTa()));
        }
        return null;
    }

    public enum Pipeline {
        APRILTAGS,
        RED_SAMPLE,
        BLUE_SAMPLE,
        YELLOW_SAMPLE,
        RED_YELLOW_SAMPLE,
        BLUE_YELLOW_SAMPLE,
        RESERVED
    }
}
