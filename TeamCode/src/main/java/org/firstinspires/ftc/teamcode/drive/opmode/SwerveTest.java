package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.drive.locolization.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.swerve.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.robot.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.util.math.Point;
import org.firstinspires.ftc.teamcode.util.math.Pose;

import java.util.function.BooleanSupplier;

@Config
@TeleOp(name = "Swerve Tele Test")
public class SwerveTest extends CommandOpMode {
    private ElapsedTime timer;
    private double loopTime = 0;

    public static double position;

    private boolean pHeadingLock = true;
    private double targetHeading;

    private final BrainSTEMRobot robot = BrainSTEMRobot.getInstance();
    private SwerveDrivetrain drivetrain;
//    private IntakeSubsystem intake;
//    private LiftSubsystem lift;

    private SlewRateLimiter fw;
    private SlewRateLimiter str;
    private final PIDFController hController = new PIDFController(0.5, 0, 0.1, 0);

    public static double fw_r = 4;
    public static double str_r = 4;
    private boolean lock_robot_heading = false;

    GamepadEx gamepadEx, gamepadEx2;
    TwoWheelLocalizer localizer;

    public static boolean autoGrabActive = false;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Constants.AUTO = false;
        Constants.USING_IMU = true;
        Constants.USE_WHEEL_FEEDFORWARD = false;

        robot.init(hardwareMap, telemetry);
        drivetrain = new SwerveDrivetrain(robot);

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        localizer = new TwoWheelLocalizer(robot);

        robot.enabled = true;

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

    }

    @Override
    public void run() {
        super.run();
        if (timer == null) {
            timer = new ElapsedTime();
            robot.reset();
            robot.startIMUThread(this);
            fw = new SlewRateLimiter(fw_r);
            str = new SlewRateLimiter(str_r);
        }

        robot.read(drivetrain);

        if (gamepad1.right_stick_button && Constants.USING_IMU)
            SwerveDrivetrain.imuOffset = robot.getAngle() + Math.PI;

        double turn = gamepad1.right_stick_x;
        if (Math.abs(turn) > 0.002) {
            lock_robot_heading = false;
        }

        double error = normalizeRadians(normalizeRadians(targetHeading) - normalizeRadians(robot.getAngle()));
        double headingCorrection = -hController.calculate(0, error) * 12.4 / robot.getVoltage();

        if (Math.abs(headingCorrection) < 0.01) {
            headingCorrection = 0;
        }

        SwerveDrivetrain.maintainHeading =
                (Math.abs(gamepad1.left_stick_x) < 0.002 &&
                        Math.abs(gamepad1.left_stick_y) < 0.002 &&
                        Math.abs(turn) < 0.002) &&
                        Math.abs(headingCorrection) < 0.02;


        double rotationAmount = (Constants.USING_IMU) ? robot.getAngle() - SwerveDrivetrain.imuOffset : 0;
        Pose DRIVE = new Pose(
                new Point(gamepad1.left_stick_y,
                        joystickScalar(gamepad1.left_stick_x, 0.001)).rotate(rotationAmount),
                lock_robot_heading ? headingCorrection :
                        turn
        );

        DRIVE = new Pose(
                fw.calculate(DRIVE.x),
                str.calculate(DRIVE.y),
                DRIVE.heading
        );


        double leftY = gamepadEx2.getRightY();
        if (Math.abs(leftY) > 0.1) {
//            intake.setSlideFactor(joystickScalar(leftY, 0.1));
        }

        if (gamepad1.a) {
            drivetrain.setLocked(true);
        } else if (gamepad1.b) {
            drivetrain.setLocked(false);
        }


        robot.loop(DRIVE, drivetrain);
        robot.write(drivetrain);
        localizer.periodic();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        drivetrain.getTelemetry();
//        telemetry.addData("height", intake.stackHeight);
        loopTime = loop;
        telemetry.update();

        robot.clearBulkCache();
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
