package org.firstinspires.ftc.teamcode.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.vision.CVMaster;
import org.firstinspires.ftc.teamcode.util.control.KalmanFilter;
import org.firstinspires.ftc.teamcode.util.hardware.GoBildaPinpoint;
import org.firstinspires.ftc.teamcode.util.hardware.Motor;

@TeleOp(name="cv tuner")
@Config
public class KalmanTuner extends LinearOpMode {
    CVMaster cv;
    GoBildaPinpoint odo;
    SparkFunOTOS otos;
    double oldTime = 0;
    Motor backLeft;
    Motor backRight;
    Motor frontLeft;
    Motor frontRight;

    public static double odoXOffset = -43.18;
    public static double odoYOffset = -22.225;
    public static double otosXOffset = 0;
    public static double otosYOffset =  6.625; //6.625
    public static double otosHeadingOffset = Math.toRadians(90);

    public static double Kpp = 0.25;
    public static double Kotos = 0.25;
    public static double Kll = 0.5;

    public double startingX = 63.75;
    public double startingY = 54.25;
    public double startingHeading = Math.toRadians(0);

    double fusedX;
    double fusedY;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        cv = new CVMaster(hardwareMap.get(Limelight3A.class, "limelight"), hardwareMap.get(WebcamName.class, "Webcam 1"));
        odo = hardwareMap.get(GoBildaPinpoint.class,"pinpoint");
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        KalmanFilter kalman = new KalmanFilter(new Pose2d(startingX, startingY, startingHeading), odo, cv.limelight);

        cv.start();
        cv.setLLPipeline(CVMaster.LLPipeline.APRILTAGS);

//        odo.resetPosAndIMU();
        odo.setOffsets(odoXOffset, odoYOffset);
        odo.recalibrateIMU();
        odo.setEncoderResolution(GoBildaPinpoint.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpoint.EncoderDirection.REVERSED, GoBildaPinpoint.EncoderDirection.FORWARD);
        odo.setPosition(new Pose2D(DistanceUnit.INCH, startingX, startingY, AngleUnit.RADIANS, startingHeading));

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(otosXOffset, otosYOffset, otosHeadingOffset);
        otos.setOffset(offset);
        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);
        otos.calibrateImu();
        otos.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(startingX, startingY, startingHeading);
        otos.setPosition(currentPosition);
        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        otos.getVersionInfo(hwVersion, fwVersion);

        backLeft = new Motor(3, "leftBack", hardwareMap, true);
        backRight = new Motor(3, "rightBack", hardwareMap, false);
        frontLeft = new Motor(3, "leftFront", hardwareMap, true);
        frontRight = new Motor(3, "rightFront", hardwareMap, false);

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            odo.update();
            Pose2D pos = odo.getPosition();
            Pose2d ppPose = new Pose2d(pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.RADIANS));
            SparkFunOTOS.Pose2D otosPos = otos.getPosition();
            Pose2d otosPose = new Pose2d(otosPos.x, otosPos.y, otosPos.h);
            LLResult result = cv.mt2RelocalizeRAW(pos.getHeading(AngleUnit.RADIANS));
            Pose3D pose3d = null;
            Pose2d llPose = null;
            if (result != null && result.isValid()) {
                pose3d = result.getBotpose_MT2();
                llPose = new Pose2d((pose3d.getPosition().x*39.3701007874), (pose3d.getPosition().y*39.3701007874), pose3d.getOrientation().getYaw(AngleUnit.RADIANS));

            }


            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;


            TelemetryPacket packet = new TelemetryPacket();
            Canvas c = packet.fieldOverlay();

            kalman.odoKalman();
            fusedX = kalman.getCalculatedState().position.x;
            fusedY = kalman.getCalculatedState().position.y;

            if (llPose != null) {
                kalman.aprilTagKalman();

                fusedX = kalman.getCalculatedState().position.x;
                fusedY = kalman.getCalculatedState().position.y;
                c.setStroke("#34ad38");
                Drawing.drawRobot(c, llPose);

//                telemetry.addData("x", llPose.position.x);
//                telemetry.addData("y", llPose.position.y);
//                telemetry.addData("heading", llPose.heading.toDouble());
//                telemetry.addData("rawPose", pose3d.toString());
//                telemetry.addData("ll", "true");

//                telemetry.addData("currentTime", System.currentTimeMillis());
//                telemetry.addData("limelight timestamp", result.getTimestamp() );
            }
            telemetry.addData("fusedX", fusedX);
            telemetry.addData("fusedY", fusedY);
            Pose2d fusedPose = new Pose2d(fusedX, fusedY, ppPose.heading.toDouble());

            c.setStroke("#edd100");
            Drawing.drawRobot(c, ppPose);

            c.setStroke("#d90209");
            Drawing.drawRobot(c, otosPose);

            c.setStroke("#fc8c03");
            Drawing.drawRobot(c, fusedPose);

            FtcDashboard.getInstance().sendTelemetryPacket(packet);

//            telemetry.addData("pose", pose);
            double otosError = Math.sqrt((Math.pow((ppPose.position.x - otosPose.position.x), 2) + Math.pow((ppPose.position.y - otosPose.position.y), 2)));
            telemetry.addData("otosError", otosError);
            telemetry.addData("px", pos.getX(DistanceUnit.INCH));
            telemetry.addData("py", pos.getY(DistanceUnit.INCH));
            telemetry.addData("pheading", pos.getHeading(AngleUnit.RADIANS));

//            telemetry.addData("looptime: ", frequency);
            telemetry.update();

            double x,y,rx;
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
}
