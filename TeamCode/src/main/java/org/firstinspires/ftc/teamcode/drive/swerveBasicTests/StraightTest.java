package org.firstinspires.ftc.teamcode.drive.swerveBasicTests;

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


import org.checkerframework.checker.index.qual.SameLen;
import org.firstinspires.ftc.teamcode.drive.locolization.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.swerve.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.robot.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.util.math.Point;
import org.firstinspires.ftc.teamcode.util.math.Pose;

import java.util.function.BooleanSupplier;

@Config
@TeleOp(name = "Straight Test")
public class StraightTest extends CommandOpMode {
    private ElapsedTime timer;
    private double loopTime = 0;

    public static double position;

    private boolean pHeadingLock = true;
    private double targetHeading;

    private final TestingBrainSTEMRobot robot = TestingBrainSTEMRobot.getInstance();

    private SampleSwerveDrive drivetrain;


    private SlewRateLimiter fw;
    private SlewRateLimiter str;
    private final PIDFController hController = new PIDFController(0.5, 0, 0.1, 0);

    public static double fw_r = 4;
    public static double str_r = 4;
    private boolean lock_robot_heading = false;

    GamepadEx gamepadEx, gamepadEx2;
    TestingTwoWheelLocalizer localizer = new TestingTwoWheelLocalizer(robot);

    public static boolean autoGrabActive = false;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Constants.AUTO = false;
        Constants.USING_IMU = true;
        Constants.USE_WHEEL_FEEDFORWARD = false;

        robot.init(hardwareMap, telemetry);

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

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

        Pose DRIVE = new Pose(
                new Point(gamepad1.left_stick_y,
                        joystickScalar(gamepad1.left_stick_x, 0.001)).rotate(0),
                0
        );

        DRIVE = new Pose(
                fw.calculate(DRIVE.x),
                str.calculate(DRIVE.y),
                DRIVE.heading
        );

        if (gamepad1.y) {
            drivetrain.setStop(false);
            drivetrain.setTurnLeft(false);
            drivetrain.setStraight(true);
        }

        if (gamepad1.x) {
            drivetrain.setStop(false);
            drivetrain.setStraight(false);
            drivetrain.setTurnLeft(true);
        }

        if (gamepad1.b) {
            drivetrain.setStraight(false);
            drivetrain.setTurnLeft(false);
            drivetrain.setStop(true);
        }

        robot.loop(DRIVE, drivetrain);
        robot.write(drivetrain);
        localizer.periodic();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));

        telemetry.addLine(drivetrain.getTelemetry());

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
