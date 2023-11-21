package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.settings.GamepadSettings;
import org.firstinspires.ftc.teamcode.util.Mechanism;
import org.firstinspires.ftc.vision.VisionPortal;

public class Drivebase extends Mechanism {

    private static final double STARTING_X = 12;
    private static double STARTING_Y = -63;
    private static double STARTING_HEADING = Math.PI/2;

    private static final double RANGE_KP = 0.02;
    private static final double YAW_KP = 0.03;

    public Camera camera;
    public MecanumDrive drive;

    final double speedClamp = 1;

    private static final double MAX_AUTO_SPEED = 0.65;
    private static final double MAX_AUTO_TURN = 0.5;

    private static final double SPEED_CLAMP_DIST = 8.0;
    private static final double DETECTION_ZONE = 24.0;

    private boolean isCameraControlling = false;

    public Drivebase(boolean isRedAlliance) {
        if (isRedAlliance) {
            STARTING_HEADING = Math.PI/2;
            STARTING_Y = -63;
        } else {
            STARTING_HEADING = 3*Math.PI/2;
            STARTING_Y = 63;
        }
    }

    @Override
    public void init(HardwareMap hwMap) {
        drive = new MecanumDrive(hwMap, new Pose2d(STARTING_X, STARTING_Y, STARTING_HEADING));
        camera = new Camera();
        camera.init(hwMap);
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (camera.getCameraState() == VisionPortal.CameraState.STREAMING) {
//            camera.checkAndSetDesiredTag(Camera.BLUE_CENTER_ID);
            camera.checkAndSetDesiredTag(-1);
        }
        updateIsCameraControlling(gamepad.a);
//        drive.setDrivePowers(clampSpeeds(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x));
        drive.setDrivePowers(stickOnly(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x));
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        camera.telemetry(telemetry);
        telemetry.addData("speedClamp", speedClamp);
        telemetry.addData("Pose Data", camera.getDesiredTagPoseData());
        telemetry.addData("Target Found", camera.targetFound);
        telemetry.addData("Current Detections", camera.getDetections());
        telemetry.addData("isCameraControlling", isCameraControlling);
    }

    public double poweredInput(double base) {
        if (GamepadSettings.EXPONENT_MODIFIER % 2 == 0) {
            return Math.pow(base, GamepadSettings.EXPONENT_MODIFIER) * Math.signum(base);
        } else {
            return Math.pow(base, GamepadSettings.EXPONENT_MODIFIER);
        }
    }

    public double stickValWithDeadzone(double stickVal) {
        if (Math.abs(stickVal) > GamepadSettings.GP1_STICK_DEADZONE) {
            return stickVal;
        } else {
            return 0;
        }
    }

    public double getDistanceToSpeedClamp(double distance) {
        return (distance - SPEED_CLAMP_DIST);
    }

    public PoseVelocity2d clampSpeeds(double y, double x, double rx) {
        double straight;
        double turn;
        double[] poseData = camera.getDesiredTagPoseData();
        if (poseData != null) {
            if (getDistanceToSpeedClamp(poseData[0]) <= DETECTION_ZONE && y >= 0 && isCameraControlling) {
                double distanceError = getDistanceToSpeedClamp(poseData[0]);
                double headingError = poseData[2];
                straight = Range.clip(distanceError * RANGE_KP, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * YAW_KP, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                return new PoseVelocity2d(
                        new Vector2d(
                                straight,
                                poweredInput(stickValWithDeadzone(x) * GamepadSettings.VX_WEIGHT)
                        ),
                        turn
                );
            } else {
                isCameraControlling = false;
            }
        }
        return stickOnly(y, x, rx);
    }

    public PoseVelocity2d stickOnly(double y, double x, double rx) {
        return new PoseVelocity2d(
                new Vector2d(
                        poweredInput(stickValWithDeadzone(y) * GamepadSettings.VY_WEIGHT),
                        poweredInput(stickValWithDeadzone(x) * GamepadSettings.VX_WEIGHT)
                ),
                poweredInput(stickValWithDeadzone(rx) * GamepadSettings.VRX_WEIGHT)
        );
    }

    public void updateIsCameraControlling(boolean isGamepadPressed) {
        double[] poseData = camera.getDesiredTagPoseData();
        if (poseData != null) {
            if (getDistanceToSpeedClamp(poseData[0]) <= DETECTION_ZONE) {
                if (isGamepadPressed) {
                    isCameraControlling = true;
                }
            }
        }
    }
}
