package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.constants.servos.clawClosedLeft;
import static org.firstinspires.ftc.teamcode.constants.servos.clawClosedRight;
import static org.firstinspires.ftc.teamcode.constants.servos.clawOpenLeft;
import static org.firstinspires.ftc.teamcode.constants.servos.clawOpenRight;

import com.qualcomm.robotcore.hardware.Servo;

public class servoCommand {

    public static void clawOpen(Servo clawLeft, Servo clawRight){
        clawLeft.setPosition(clawOpenLeft);
        clawRight.setPosition(clawOpenRight);
    }

    public static void clawClose(Servo clawLeft, Servo clawRight){
        clawLeft.setPosition(clawClosedLeft);
        clawRight.setPosition(clawClosedRight);
    }
}
