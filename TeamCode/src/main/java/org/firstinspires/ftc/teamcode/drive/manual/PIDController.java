package org.firstinspires.ftc.teamcode.drive.opmode.manual;


import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private final double Kp, Ki, Kd;

    double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime timer;
    private double maxIntegral = 500;

    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.timer = new ElapsedTime();
    }

    public double calculate(double reference, double state) {
        double error = reference - state;

        double velocityError = applyDeadZone(error, 5);

        double deltaTime = timer.seconds();
        timer.reset();

        if (Math.abs(reference) < 0.1) {
            integralSum = 0;
        }
        else {
            integralSum += velocityError * deltaTime;
        }

        integralSum = Math.max(Math.min(integralSum, maxIntegral), -maxIntegral);

        double derivative = deltaTime > 0 ? (velocityError - lastError) / deltaTime : 0;

        lastError = velocityError;

        return (Kp * velocityError) + (Ki * integralSum) + (Kd * derivative);
    }

    private double applyDeadZone(double value, double deadZone) {
        return (Math.abs(value) < deadZone) ? 0 : value;
    }
}
