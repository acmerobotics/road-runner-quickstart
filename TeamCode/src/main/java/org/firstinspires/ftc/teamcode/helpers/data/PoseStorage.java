package org.firstinspires.ftc.teamcode.helpers.data;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class PoseStorage {

    private static final String POSE_FILE_NAME = "LastPose.txt";

    // Save the robot's pose to a file
    public static void savePose(Pose2d pose) {
        File file = AppUtil.getInstance().getSettingsFile(POSE_FILE_NAME);
        try {
            String data = pose.position.x + "," + pose.position.y + "," + pose.heading.toDouble();
            ReadWriteFile.writeFile(file, data);
        } catch (Exception e) {
            // Handle exceptions if necessary
        }
    }

    // Load the robot's pose from a file
    public static Pose2d loadPose() {
        File file = AppUtil.getInstance().getSettingsFile(POSE_FILE_NAME);
        try {
            String data = ReadWriteFile.readFile(file);
            String[] tokens = data.split(",");
            if (tokens.length == 3) {
                double x = Double.parseDouble(tokens[0]);
                double y = Double.parseDouble(tokens[1]);
                double heading = Double.parseDouble(tokens[2]);
                return new Pose2d(x, y, heading);
            } else {
                return null;
            }
        } catch (Exception e) {
            return null;
        }
    }
}
