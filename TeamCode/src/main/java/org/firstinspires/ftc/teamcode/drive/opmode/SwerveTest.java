package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.locolization.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.swerve.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.robot.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.util.math.Point;
import org.firstinspires.ftc.teamcode.util.math.Pose;

//@Disabled
@TeleOp(name = "Localization Test (swerve) ", group = "drive")
public class SwerveTest extends LinearOpMode {

    TwoWheelLocalizer swerveLocolizer;

    private BrainSTEMRobot robot = BrainSTEMRobot.getInstance();


    private ElapsedTime timer;

    private SwerveDrivetrain swerveDrive;

    private SlewRateLimiter forward;

    private SlewRateLimiter strafe;

    private boolean lock_robot_heading = false;

    @Override
    public void runOpMode() throws InterruptedException {


        swerveLocolizer = new TwoWheelLocalizer(robot);






        robot.reset();

        waitForStart();

        while (opModeInInit()) {

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            Constants.AUTO = false;
            Constants.USING_IMU = true;
            Constants.USE_WHEEL_FEEDFORWARD = false;

            robot.init(hardwareMap, telemetry);

            robot.enabled = true;

            swerveLocolizer.setPoseEstimate(new Pose2d(0,0,0));
            robot.startIMUThread(this);

            forward = new SlewRateLimiter(5);
            strafe = new SlewRateLimiter(5);


            PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            PhotonCore.experimental.setMaximumParallelCommands(8);
            PhotonCore.enable();
        }

        while (!isStopRequested()) {

            robot.read(swerveDrive);

            double turn = gamepad1.right_stick_x;

            double headingCorrection = -hController.calculate(0, error) * 12.4 / robot.getVoltage();

            if (Math.abs(headingCorrection) < 0.01) {
                headingCorrection = 0;
            }


            double rotationAmount = (Constants.USING_IMU) ? robot.getAngle() - SwerveDrivetrain.imuOffset : 0;
            Pose drive = new Pose(
                    new Point(joystickScalar(gamepad1.left_stick_y, 0.001),
                            joystickScalar(gamepad1.left_stick_x, 0.001)).rotate(rotationAmount),
                    lock_robot_heading ? headingCorrection :
                            joystickScalar(turn, 0.01)
            );

            drive = new Pose(
                    forward.calculate(drive.x),
                    strafe.calculate(drive.y),
                    drive.heading
            );

            swerveLocolizer.periodic();
            robot.loop(drive, swerveDrive);
            robot.write(swerveDrive);


            Pose poseEstimate = swerveLocolizer.getPos();
            telemetry.addData("X Estimate :", poseEstimate.x);
            telemetry.addData("Y Estimate :", poseEstimate.y);
            telemetry.addData("Heading Estimate :", poseEstimate.heading);
            telemetry.addLine("--------------------");
            telemetry.update();

            robot.clearBulkCache();
        }
    }

    private double joystickScalar(double num, double min) {
        return joystickScalar(num, min, 0.66, 3);
    }

    private double joystickScalar(double n, double m, double l, double a) {
        return Math.signum(n) * m
                + (1 - m) *
                (Math.abs(n) > l ?
                        Math.pow(Math.abs(n), Math.log(l / a) / Math.log(l)) * Math.signum(n) :
                        n / a);
    }

}
