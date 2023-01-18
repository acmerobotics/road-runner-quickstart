package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public final class constants {

    public static final class servos{
        public static final double clawOpenLeft = 0.005;
        public static final double clawOpenRight = 0.005;
        public static final double clawClosedLeft = 0.025;
        public static final double clawClosedRight = 0.025;
    }

    public static final class slides{
        public static final double ticksToInches = 200;
        public static final int[] slidePosArray = { 0, 1715, 3055, 4355 };

    public static final double slideOnePID[] = {1, 0.0, 0.2};
    public static final double slideTwoPID[] = {1, 0.0, 0.2};
    }


    public static final class motors{
        DcMotor lF = hardwareMap.dcMotor.get("front_left");
        DcMotor lB = hardwareMap.dcMotor.get("back_left");
        DcMotor rF = hardwareMap.dcMotor.get("front_right");
        DcMotor rB = hardwareMap.dcMotor.get("back_right");
        DcMotor slideOne = hardwareMap.dcMotor.get("slide_one");
        Servo clawLeft = hardwareMap.servo.get("claw_left");
        Servo clawRight = hardwareMap.servo.get("claw_right");
    }

    public static final class drive{
        public static final double driveSpeed = 0.7;
    }
}