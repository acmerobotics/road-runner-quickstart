package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Mechanism;
public class clawDustpan extends Mechanism {

        Servo clawDustpan;

        public String intakeName = "clawDustpan";

        @Override
        public void init(HardwareMap hwMap) {
            clawDustpan = hwMap.get(Servo.class, intakeName);
            }

        @Override
        public void loop(Gamepad gamepad) {
            if (gamepad.a) {
                clawDustpan.setPosition(1);
            } else if (gamepad.b) {
                clawDustpan.setPosition(-1);
            } else {
                clawDustpan.setPosition(0);
            }
        }
    }


