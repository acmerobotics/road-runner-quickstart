package org.firstinspires.ftc.teamcode.robot.swerve;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import static org.firstinspires.ftc.teamcode.robot.Constants.*;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.util.hardware.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.util.math.MathUtils;
import org.firstinspires.ftc.teamcode.util.math.Pose;

@Config
public class SwerveDrivetrain implements Drivetrain {
    public SwerveModule frontLeftModule, backLeftModule, backRightModule, frontRightModule;
    public SwerveModule[] modules;

    public static double TRACK_WIDTH = 9, WHEEL_BASE = 9;
    private final double R;
    public static double frontLeftOffset = -0.05, frontRightOffset = 0, backLeftOffset = 0, backRightOffset = -0.055;

    public static boolean maintainHeading = false;

    double[] motor = new double[4];
    double[] crServo = new double[4];
    double max = 0.0;

    public final double minPow = 0.1;
    public static double imuOffset = 0.0;

    private boolean locked = false;

    private boolean restPosition = false;

    public SwerveDrivetrain(RobotHardware robot) {
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

        if (locked) {
            motor = new double[]{0, 0, 0, 0};
            crServo = new double[]{Math.PI / 4, -Math.PI / 4, Math.PI / 4, -Math.PI / 4};
        } else if (restPosition) {
            motor = new double[]{0, 0, 0, 0};
            crServo = new double[]{0, 0, 0, 0};
        } else {
            motor = new double[]{hypot(b, c), hypot(b, d), hypot(a, d), hypot(a, c)};
            if (!maintainHeading) crServo = new double[]{atan2(b, c), atan2(b, d), atan2(a, d), atan2(a, c)};
        }

        max = MathUtils.max(motor);
    }

    public void write() {
        for (int i = 0; i < 4; i++) {
            SwerveModule modules = this.modules[i];
            if (Math.abs(max) > 1) motor[i] /= max;
            modules.setMotorPower(Math.abs(motor[i]) + ((USE_WHEEL_FEEDFORWARD) ? minPow * Math.signum(motor[i]) : 0));
            modules.setTargetRotation(MathUtils.norm(crServo[i]));
        }
    }

    public void updateModules() {
        for (SwerveModule m : modules) m.update();
    }

    public void setLocked(boolean locked){
        this.locked = locked;
    }

    public void setReset(boolean restPosition) {
        this.restPosition = restPosition;
    }

    public boolean isLocked(){
        return locked;
    }

    public boolean isReset(){
        return restPosition;
    }

    public String getTelemetry() {
        return frontLeftModule.getTelemetry("leftFrontModule") + "\n" +
                backLeftModule.getTelemetry("leftRearModule") + "\n" +
                frontRightModule.getTelemetry("rightFrontModule") + "\n" +
                backRightModule.getTelemetry("rightRearModule") + "\n";
    }
}