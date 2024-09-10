package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.nio.channels.Pipe;
import java.util.EnumMap;

public class CVMaster {
    public Limelight3A limelight;
    public Relocalization relocalization;
    EnumMap<Pipeline, Integer> pipelines = new EnumMap<>(Pipeline.class);

    public CVMaster(Limelight3A ll3a) {
        limelight = ll3a;
        relocalization = new Relocalization();

        pipelines.put(Pipeline.APRILTAGS, 0);
        pipelines.put(Pipeline.RED_SAMPLE, 1);
        pipelines.put(Pipeline.BLUE_SAMPLE, 2);
        pipelines.put(Pipeline.RED_YELLOW_SAMPLE, 3);
        pipelines.put(Pipeline.BLUE_YELLOW_SAMPLE, 4);
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

    public Pose2d relocalizeBlind() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D pose = result.getBotpose();
            return new Pose2d(pose.getPosition().x,
                    pose.getPosition().y,
                    Rotation2d.fromDegrees(pose.getOrientation().getPitch(AngleUnit.DEGREES)));
        }

        return null;
    }

    public enum Pipeline {
        APRILTAGS,
        RED_SAMPLE,
        BLUE_SAMPLE,
        RED_YELLOW_SAMPLE,
        BLUE_YELLOW_SAMPLE
    }
}
