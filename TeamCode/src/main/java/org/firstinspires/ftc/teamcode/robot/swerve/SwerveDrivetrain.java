package org.firstinspires.ftc.teamcode.robot.swerve;

import static org.firstinspires.ftc.teamcode.robot.Constants.USE_WHEEL_FEEDFORWARD;
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.util.hardware.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.util.math.MathUtils;
import org.firstinspires.ftc.teamcode.util.math.Pose;

@Config
public class SwerveDrivetrain implements Drivetrain {
    public SwerveModule frontLeftModule, backLeftModule, backRightModule, frontRightModule;
    public SwerveModule[] modules;
    public static double TRACK_WIDTH = 9, WHEEL_BASE = 9;
    private final double R;

    public static boolean maintainHeading = false;

    public static double frontLeftOffset = 0, frontRightOffset = 0, backLeftOffset = 0, backRightOffset = 0;

    double[] ws = new double[4];
    double[] wa = new double[4];
    double max = 0.0;

    public final double minPow = 0.1;
    public static double imuOffset = 0.0;

    private boolean forward = false;

    public SwerveDrivetrain(BrainSTEMRobot robot) {
        frontLeftModule = new SwerveModule(robot.frontLeftMotor, robot.frontLeftServo, new AbsoluteAnalogEncoder(robot.frontLeftEncoder, 3.3).zero(frontLeftOffset).setInverted(true));
        backLeftModule = new SwerveModule(robot.backLeftMotor, robot.backLeftServo, new AbsoluteAnalogEncoder(robot.backLeftEncoder, 3.3).zero(backLeftOffset).setInverted(true));
        backRightModule = new SwerveModule(robot.backRightMotor, robot.backRightServo, new AbsoluteAnalogEncoder(robot.backRightEncoder, 3.3).zero(backRightOffset).setInverted(true));
        frontRightModule = new SwerveModule(robot.frontRightMotor, robot.frontRightServo, new AbsoluteAnalogEncoder(robot.frontRightEncoder, 3.3).zero(frontRightOffset).setInverted(true));

        modules = new SwerveModule[]{frontLeftModule, frontRightModule, backRightModule, backLeftModule};
        for (SwerveModule m : modules) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        R = hypot(TRACK_WIDTH, WHEEL_BASE);
    }

    public void read() {
        for (SwerveModule module : modules) module.read();
    }


    @Override
    public void set(Pose pose) {
        double x = pose.x, y = pose.y, head = pose.heading;

        double a = x - head * (WHEEL_BASE / R),
                b = x + head * (WHEEL_BASE / R),
                c = y - head * (TRACK_WIDTH / R),
                d = y + head * (TRACK_WIDTH / R);

        if (forward) {
            ws = new double[]{0.1, 0.1, 0.1, 0.1};
            wa = new double[]{0, 0, 0, 0};
        } else {
            ws = new double[]{hypot(b, c), hypot(b, d), hypot(a, d), hypot(a, c)};
            if (!maintainHeading) wa = new double[]{atan2(b, c), atan2(b, d), atan2(a, d), atan2(a, c)};
        }

        max = MathUtils.max(ws);
    }

    public void write() {
        for (int i = 0; i < 4; i++) {
            SwerveModule m = modules[i];
            if (Math.abs(max) > 1) ws[i] /= max;
            m.setMotorPower(Math.abs(ws[i]) + ((USE_WHEEL_FEEDFORWARD) ? minPow * Math.signum(ws[i]) : 0));
//            m.setMotorPower(0);
            m.setTargetRotation((MathUtils.norm(wa[i])) );
        }
    }

    public void updateModules() {
        for (SwerveModule m : modules) m.update();
    }

    public void setForward(boolean forward){
        this.forward = forward;
    }

    public boolean isForward(){
        return forward;
    }

    public String getTelemetry() {
        return frontLeftModule.getTelemetry("leftFrontModule") + "\n" + "-----------"  + "\n" +
                backLeftModule.getTelemetry("leftRearModule") + "\n" + "-----------"  + "\n" +
                frontRightModule.getTelemetry("rightFrontModule") + "\n" + "-----------"  + "\n" +
                backRightModule.getTelemetry("rightRearModule") + "\n" + "-----------";
    }
}