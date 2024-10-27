package org.firstinspires.ftc.teamcode.az.sample;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;


@Autonomous
public class Gripper extends LinearOpMode {

    private Servo wrist;
    private CRServo roller;
    LinearOpMode opMode;

    public Gripper() {
        super();
    }

    public Gripper(LinearOpMode newOpMode) {
        this.opMode = newOpMode;
        setup();
    }



    public enum GripperPos {

        WRIST_SAMPLE_DROP(0.5),

        WRIST_SPECIMENT_DROP(0),
        WRISTRESET(0),

        WRIST_MOVE(0.5),

        WRISTCOLLECT(0.5),

        ROLLERDROP(0.5),
        ROLLERCOLLECT(-0.5);


        private double value;

        GripperPos(double val) {
            this.value = val;
        }

        public double getValue() {
            return this.value;
        }
    }

    public void setup() {
        wrist = opMode.hardwareMap.get(Servo.class, "wrist");
        wrist.setDirection(Servo.Direction.REVERSE);

        roller = opMode.hardwareMap.get(CRServo.class, "roller");


        setupPos();
    }

    public void setupPos() {
        reset();
    }


    public void sampleDrop() {
        roller.setPower(GripperPos.ROLLERDROP.value);
        wrist.setPosition(GripperPos.WRIST_SAMPLE_DROP.value);
    }

    public void move() {
        roller.setPower(0);
        wrist.setPosition(GripperPos.WRIST_MOVE);
    }

    public void collect() {
        roller.setPower(GripperPos.ROLLERCOLLECT.value);
        wrist.setPosition(GripperPos.WRISTCOLLECT.value);
    }

    public void reset() {
        wrist.setPosition(GripperPos.WRISTRESET.value);
        roller.setPower(0);
    }

    @Override
    public void runOpMode() {
        this.opMode = this;

        telemetry.addLine("Init");
        telemetry.update();

        setup();
        waitForStart();

        while (opModeIsActive()) {

            reset();
            sleep(2000);

            collect();
            sleep(5000);

            sampleDrop();
            sleep(5000);
        }
    }
}