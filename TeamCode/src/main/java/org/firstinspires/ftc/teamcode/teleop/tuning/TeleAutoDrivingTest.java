package org.firstinspires.ftc.teamcode.teleop.tuning;

import static org.firstinspires.ftc.teamcode.Robot.normalizeRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.roadrunner.KalmanDrive;
import org.firstinspires.ftc.teamcode.subsystems.vision.CVMaster;
import org.firstinspires.ftc.teamcode.util.hardware.Motor;
import org.firstinspires.ftc.teamcode.util.misc.FullPose2d;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="auto driving")
@Config
public class TeleAutoDrivingTest extends LinearOpMode {
    DrivingMode drivingMode = DrivingMode.MANUAL;
    CVMaster cv;
    double oldTime = 0;
    Motor backLeft;
    Motor backRight;

    boolean oldCross = false;
    boolean oldCircle = false;

    Motor frontLeft;
    Motor frontRight;

    KalmanDrive drive;
    Pose3D target = new Pose3D(new Position(DistanceUnit.INCH, -50, -50, 0, System.currentTimeMillis()), new YawPitchRollAngles(AngleUnit.RADIANS, 0,0,0,System.currentTimeMillis()));


    @Override
    public void runOpMode() throws InterruptedException {
        List<Action> actionsQueue = new ArrayList<>();
        cv = new CVMaster(hardwareMap.get(Limelight3A.class, "limelight"), hardwareMap.get(WebcamName.class, "Webcam 1"));
        drive = new KalmanDrive(hardwareMap, new Pose2d(0,0,0), cv.limelight);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        cv.start();
        cv.setLLPipeline(CVMaster.LLPipeline.APRILTAGS);

        backLeft = new Motor(3, "leftBack", hardwareMap, true);
        backRight = new Motor(3, "rightBack", hardwareMap, false);
        frontLeft = new Motor(3, "leftFront", hardwareMap, true);
        frontRight = new Motor(3, "rightFront", hardwareMap, false);

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();
            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;

            drive.updatePoseEstimate();

            if (drivingMode == DrivingMode.MANUAL) {
                if (gamepad1.a && !oldCross) {
                    FullPose2d robotTargetPose = cv.calculateRobotFullPose(target, drive.pose.position.x, drive.pose.position.y);
                    if (robotTargetPose.intakeExtension > 15) {
                        // WHEN JUST TURNING ISNT ENOUGH FOR THE BOT TO REACH THE SAMPLE
                        double normalHeading = normalizeRadians(drive.pose.heading.toDouble());
                        if (normalHeading > ((3 * Math.PI) / 4) && normalHeading < ((5 * Math.PI) / 4)) {
                            // ZONE I (Facing Right)
                            robotTargetPose = cv.calculateRobotFullPose(target, drive.pose.position.x, target.getPosition().y);
                        } else if (normalHeading > ((7 * Math.PI) / 4) || normalHeading < (Math.PI / 4)) {
                            // ZONE II (Facing Left)
                            robotTargetPose = cv.calculateRobotFullPose(target, drive.pose.position.x, target.getPosition().y);
                        } else if (normalHeading > ((5 * Math.PI) / 4) && normalHeading < ((7 * Math.PI) / 4)) {
                            // ZONE III (Facing Down)
                            robotTargetPose = cv.calculateRobotFullPose(target, target.getPosition().x, drive.pose.position.y);
                        } else if (normalHeading > (Math.PI / 4) && normalHeading < ((3 * Math.PI) / 4)) {
                            // ZONE IV (Facing Up)
                            robotTargetPose = cv.calculateRobotFullPose(target, target.getPosition().x, drive.pose.position.y);
                        }
                    }

                    if (Math.sqrt(Math.pow((drive.pose.position.x - robotTargetPose.getRobotPose().position.x), 2) + Math.pow((drive.pose.position.y - robotTargetPose.getRobotPose().position.y), 2)) > 30
                            || robotTargetPose.getRobotPose().position.x + 6 > 60 || robotTargetPose.getRobotPose().position.x - 6 < -60
                            || robotTargetPose.getRobotPose().position.y - 7 < -60 || robotTargetPose.getRobotPose().position.y + 7 > 60
                            || Math.abs(robotTargetPose.getRobotPose().heading.toDouble() - drive.pose.heading.toDouble()) > (Math.PI)) {
                        // FAILSAFE TO STOP THE BOT FROM TRYING TO GO TO SOME CRAZY AHH LOCATION
                        // CHECKS ARE: TRYING TO MOVE MORE THAN 30in; ROBOT PARTS WILL BE OUT OF BOUNDS; TRYING TO ROTATE MORE THAN 180deg
                        // TODO: ACCOUNT FOR HEADING IN OUT OF BOUNDS CALCS
                        actionsQueue.add(new InstantAction(() -> {
                            gamepad1.rumbleBlips(5);
                            gamepad1.runLedEffect(new Gamepad.LedEffect.Builder()
                                    .addStep(255,0,0, 150)
                                    .addStep(0,0,0,150)
                                    .addStep(255,0,0, 150)
                                    .addStep(0,0,0,150)
                                    .addStep(255,0,0, 150)
                                    .addStep(0,0,0,150)
                                    .addStep(255,0,0, 150)
                                    .addStep(0,0,0,150)
                                    .build()
                            );
                        }));

                    } else {
                        Action path;
                        Action pathBack;
                        if (drive.pose.position.equals(robotTargetPose.getRobotPose().position)) {
                            path = drive.actionBuilder(drive.pose)
                                    .turnTo(robotTargetPose.getRobotPose().heading)
                                    .build();
                            pathBack = drive.actionBuilder(robotTargetPose.getRobotPose())
                                    .setReversed(true)
                                    .turnTo(drive.pose.heading)
                                    .build();
                        } else {
                            path = drive.actionBuilder(drive.pose)
                                    .splineToLinearHeading(robotTargetPose.getRobotPose(), drive.pose.heading)
                                    .build();
                            pathBack = drive.actionBuilder(robotTargetPose.getRobotPose())
                                    .setReversed(true)
                                    .splineToLinearHeading(drive.pose, drive.pose.heading)
                                    .build();
                        }

                        actionsQueue.add(new SequentialAction(
                                new InstantAction(() -> drivingMode = DrivingMode.AUTOMATIC),
                                new InstantAction(() -> gamepad1.rumbleBlips(2)),
                                path,
                                new SleepAction(1.5),
                                pathBack,
                                new InstantAction(() -> drivingMode = DrivingMode.MANUAL)
                        ));
                    }
                }

                if (gamepad1.b && !oldCircle) {
                    target = new Pose3D(new Position(DistanceUnit.INCH, drive.pose.position.x, drive.pose.position.y, 0, System.currentTimeMillis()), new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, System.currentTimeMillis()));
                }

                double x, y, rx;
                if (gamepad1.right_trigger > 0.5) {
                    x = -gamepad1.left_stick_x * (1 - 0.66 * gamepad1.right_trigger);
                    y = -gamepad1.left_stick_y * (1 - 0.66 * gamepad1.right_trigger);
                    rx = gamepad1.right_stick_x * (1 - 0.66 * gamepad1.right_trigger);

                } else {
                    x = -gamepad1.left_stick_x;
                    y = -gamepad1.left_stick_y;
                    rx = gamepad1.right_stick_x;
                }
                setDrivePower(-x, y, rx);
            } else if (drivingMode == DrivingMode.AUTOMATIC) {
                // nothing rn bc no cancels

                if (Math.abs(gamepad1.right_stick_x) > 0.1 || Math.abs(gamepad1.right_stick_y) > 0.1 || Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_y) > 0.1) {
                    gamepad1.rumble(250);
                }
            }

            List<Action> newActions = new ArrayList<>();
            for (Action action : actionsQueue) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            actionsQueue = newActions;

            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            telemetry.addData("looptime: ", frequency);
            telemetry.addData("MODE", drivingMode);
            telemetry.addData("TARGET", target.getPosition().toString());
            telemetry.addData("ACTIONS QUEUE", actionsQueue.size());
            telemetry.addData("NEW CIRCLE", gamepad1.a);
            telemetry.addData("OLD CIRCLE", oldCross);
            telemetry.update();

            oldCircle = gamepad1.b;
            oldCross = gamepad1.a;
        }
    }

    public void setDrivePower(double x, double y, double rx) {
        double powerFrontLeft = y + x + rx;
        double powerFrontRight = y - x - rx;
        double powerBackLeft = (y - x + rx) * -1;
        double powerBackRight = (y + x - rx) * -1;

        if (Math.abs(powerFrontLeft) > 1 || Math.abs(powerBackLeft) > 1 ||
                Math.abs(powerFrontRight) > 1 || Math.abs(powerBackRight) > 1) {
            // Find the largest power
            double max;
            max = Math.max(Math.abs(powerFrontLeft), Math.abs(powerBackLeft));
            max = Math.max(Math.abs(powerFrontRight), max);
            max = Math.max(Math.abs(powerBackRight), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            powerFrontLeft /= max;
            powerBackLeft /= max;
            powerFrontRight /= max;
            powerBackRight /= max;
        }

        frontLeft.setSpeed((float)powerFrontLeft);
        frontRight.setSpeed((float)powerFrontRight);
        backLeft.setSpeed(-(float)powerBackLeft);
        backRight.setSpeed(-(float)powerBackRight);
    }

    enum DrivingMode {
        MANUAL,
        AUTOMATIC,
        DISABLED
    }
}
