package org.firstinspires.ftc.teamcode.huskyteers.vision;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class SideAprilTagDetection {
//
//    public SideAprilTagDetection(HardwareMap hwMap) {
//    for (AprilTagDetection detection : aprilTag.getDetections())  {
//
//     Orientation rot = Orientation.getOrientation(detection.rawPose.R, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
//
//     // Data from Apriltag detection
//     double poseX = detection.rawPose.x;
//     double poseY = detection.rawPose.y;
//     double poseZ = detection.rawPose.z;
//
//     double poseAX = rot.firstAngle;
//     double poseAY = rot.secondAngle;
//     double poseAZ = rot.thirdAngle;
//     }
//    if (detection.rawPose != null)   {
//     detection.ftcPose = new AprilTagPoseFtc();
//
//     detection.ftcPose.x =  detection.rawPose.x;
//     detection.ftcPose.y =  detection.rawPose.z;
//     detection.ftcPose.z = -detection.rawPose.y;
//
//     Orientation rot = Orientation.getOrientation(detection.rawPose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, outputUnitsAngle);
//     detection.ftcPose.yaw = -rot.firstAngle;
//     detection.ftcPose.roll = rot.thirdAngle;
//     detection.ftcPose.pitch = rot.secondAngle;
//
//     detection.ftcPose.range = Math.hypot(detection.ftcPose.x, detection.ftcPose.y);
//     detection.ftcPose.bearing = outputUnitsAngle.fromUnit(AngleUnit.RADIANS, Math.atan2(-detection.ftcPose.x, detection.ftcPose.y));
//     detection.ftcPose.elevation = outputUnitsAngle.fromUnit(AngleUnit.RADIANS, Math.atan2(detection.ftcPose.z, detection.ftcPose.y));
//     }
//    }
}
