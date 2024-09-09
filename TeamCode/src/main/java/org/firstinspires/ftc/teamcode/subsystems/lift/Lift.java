package org.firstinspires.ftc.teamcode.subsystems.lift;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.util.Levels;
import org.firstinspires.ftc.teamcode.util.MotionProfile;
import org.firstinspires.ftc.teamcode.util.MotionProfileGenerator;
import org.firstinspires.ftc.teamcode.util.Motor;

import java.util.concurrent.TimeUnit;
import java.util.function.Function;

public class Lift {
    private PIDController controller1;

    private MotionProfile profile;
    private ElapsedTime timer;
    double maxvel = 6000;
    double maxaccel = 6000;

    public double p = 0.01, i = 0.00, d = 0.0001;
    public double f = 0.006;
    double voltageCompensation;

    public double target = 0;
    public Levels currentLevel = Levels.ZERO;
    private final double ticks_in_degrees = 700 / 180.0;
    public double power1;

    public Motor slides1;
    public Motor slides2;
    public VoltageSensor voltageSensor;

    private boolean threadState = false;


    public Lift(Motor l1, Motor l2, VoltageSensor voltageSensor) {
        this.slides1 = l1;
        this.slides2 = l2;
        this.voltageSensor = voltageSensor;

        controller1 = new PIDController(p, i , d);
        slides1.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        timer = new ElapsedTime();
        profile = MotionProfileGenerator.generateSimpleMotionProfile(1, 0, maxvel, maxaccel);
    }


    public void update() {
        target = profile.get(timer.time());

        int motorPos = slides1.motor.getCurrentPosition();

        double pid1 = controller1.calculate(motorPos, target);
//        double pid2 = controller2.calculate(slides2Pos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        voltageCompensation = 13.2 / voltageSensor.getVoltage();
        power1 = (pid1 + ff) * voltageCompensation;
//        power2 = pid2 + ff;

        if (target == 0){
            slides1.motor.setPower(power1);
            slides2.motor.setPower(power1); //was at *0.3 pre push
        }
        else {
            slides1.motor.setPower(power1);
            slides2.motor.setPower(power1);
        }
    }

    public void runToPosition(int ticks) {
//        target = ticks;
        profile = MotionProfileGenerator.generateSimpleMotionProfile(getPos(), ticks, maxvel, maxaccel);
        timer.reset();
    }

    public void runToPreset(Levels preset) {
        if (preset == Levels.ZERO) {
            runToPosition(0);
        } else if (preset == Levels.INTAKE) {
            runToPosition(0);
        } else if (preset == Levels.DEPOSIT){
            runToPosition(0);
        }
    }

    public void setPower(float power) {
        slides1.motor.setPower(power);
        slides2.motor.setPower(power);
    }

    public void setPower(float power1, float power2) {
        slides1.motor.setPower(power1);
        slides2.motor.setPower(power2);
    }

    public void launchAsThread(Telemetry telemetry) {
        threadState = true;
        telemetry.addData("Slides Threads State:", "STARTING");
        telemetry.update();
        Thread t1 = new Thread(() -> {
            telemetry.addData("Slides Threads State:", "STARTED");
            telemetry.update();
            while (threadState == true) {
                update();
            }
            telemetry.addData("Slides Threads State:", "STOPPED");
            telemetry.update();
        });
        t1.start();
    }

    public void destroyThreads(Telemetry telemetry) {
        telemetry.addData("Slides Threads State:", "STOPPING");
        telemetry.update();
        target = -10;
        threadState = false;
    }

    public void launchAsThreadBasic() {
        threadState = true;
        Thread t1 = new Thread(() -> {
            while (threadState == true) {
                update();
            }
        });
        t1.start();
    }

    public void destroyThreadsBasic() {
        threadState = false;
    }
    public void resetAllEncoders(){
        slides1.resetEncoder();
        slides2.resetEncoder();
    }

    public int getPos() {
        return slides1.motor.getCurrentPosition();
    }

}