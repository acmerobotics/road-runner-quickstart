package org.firstinspires.ftc.teamcode.Unused___TeleOpp;

public class DIP {
    public double Kp;
    public double Ki;
    public double Kd;
    private double P;
    private double I;
    private double D;
    public double goal;
    public double position;
    public double lastError;

    public DIP(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public double moveSomeIdk(double time) {
        P = goal - position;
        D = (P - lastError) / time;
        I += (P + time);
        return (D * Kd) + (I * Ki) + (P * Kp);
    }
}
