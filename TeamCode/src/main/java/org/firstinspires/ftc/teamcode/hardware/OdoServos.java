package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.C;
@Config
public class OdoServos extends ServoMechanism{
    private double leftPos = .25;
    private double rightPos = .25;
    private double frontPos = .25;
    private Servo left, right, front;

    @Override
    public void init(HardwareMap hwMap) {
        left = hwMap.servo.get("leftServo");
        right = hwMap.servo.get("rightServo");
        front = hwMap.servo.get("frontServo");
    }

    @Override
    public void run(boolean bool) {
        if(bool) {
            left.setPosition(leftPos);
            right.setPosition(rightPos);
            front.setPosition(frontPos);
        }else {
            left.setPosition(0);
            right.setPosition(0);
            front.setPosition(0);
        }
    }
}
