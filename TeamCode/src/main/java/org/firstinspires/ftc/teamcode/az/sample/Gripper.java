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

        WRIST_SPECIMEN_HANG(0),
        WRISTHALFWAYRESET(0.2),
        WRISTRESET(0),

        WRIST_MOVE(0.5),

        WRISTCOLLECT(0.5),

        ROLLERDROP(1),
        ROLLERCOLLECT(-1);


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
        sleep(200);
        //wrist.setPosition(GripperPos.WRIST_SAMPLE_DROP.value);
    }

    public void move() {
        roller.setPower(0);
        wrist.setPosition(GripperPos.WRIST_MOVE.value);
    }

    public void collect() {
        roller.setPower(GripperPos.ROLLERCOLLECT.value);
        wrist.setPosition(GripperPos.WRISTCOLLECT.value);
    }

    public void reset() {
        wrist.setPosition(GripperPos.WRISTRESET.value);
        roller.setPower(0);
    }

    public void halfwayReset() {
        wrist.setPosition(GripperPos.WRISTHALFWAYRESET.value);
        roller.setPower(0);
    }

    public void specimenHang(){
        roller.setPower(0);
        wrist.setPosition(GripperPos.WRIST_SPECIMEN_HANG.value);
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