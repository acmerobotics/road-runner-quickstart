package org.firstinspires.ftc.teamcode.azutils;

public interface IRobot {
    void moveForward(double power);
    void moveReverse(double power);
    void turnRight(double power);
    void turnLeft(double power);
    void stop();

    //PID
    double getKp();
    double getKi();
    double getKd();
}
