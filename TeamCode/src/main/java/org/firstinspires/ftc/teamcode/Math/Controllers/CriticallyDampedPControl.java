package org.firstinspires.ftc.teamcode.Math.Controllers;

import org.firstinspires.ftc.teamcode.Math.AsymmetricProfile.MotionState;

/**
 * a feedback / feedforward controller that uses the linear system model: V = Kv * V + Ka + A in
 * order to synthesize the proportional gain using pole placement to find the critically damped pole
 */
public class CriticallyDampedPControl {

    double Kv;
    double Ka;
    double Kp;

    /**
     * Initialize the critically damped controller
     * @param Kv velocity gain
     * @param Ka acceleration gain
     */
    public CriticallyDampedPControl(double Kv, double Ka) {
        this.Ka = Ka;
        this.Kv = Kv;
        this.Kp = Math.pow(Kv,2) / 4 * Ka; // highest value of K that has two real poles.
    }

    public double calculate(MotionState reference, MotionState state) {
        double feedforward = reference.getV() * Kv + reference.getA() * Ka;
        double feedback = (reference.getX() - state.getX()) * Kp;
        return feedback + feedforward;
    }
}
