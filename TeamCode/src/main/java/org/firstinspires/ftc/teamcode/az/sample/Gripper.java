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



    public enum RollerPower {
        COLLECT(-1),

        DROP(1);
        private double value;

        RollerPower(double val) {
            this.value = val;
        }

        public double getValue() {
            return this.value;
        }
    }

    public enum GripperPos {

        WRIST_SAMPLE_DROP(0.5),

        WRIST_SPECIMEN_HANG(0),
        WRISTHALFWAYRESET(0.2),
        WRISTRESET(0),

        WRIST_MOVE(0.5),

        WRISTCOLLECT(0.55),

        AUTO_WRISTCOLLECT(.75),

        ROLLERDROP(0.55),
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
        roller.setPower(RollerPower.DROP.value);
        //sleep(200);
        //wrist.setPosition(GripperPos.WRIST_SAMPLE_DROP.value);
    }

    public void moveAround() {
        wrist.setPosition(GripperPos.WRISTCOLLECT.value );
        sleep(1000);
        wrist.setPosition(GripperPos.WRISTCOLLECT.value - 0.4);
        sleep(500);
        wrist.setPosition(GripperPos.WRISTCOLLECT.value + 0.4);
        sleep(500);
        wrist.setPosition(GripperPos.WRISTCOLLECT.value );
        sleep(500);
    }


    public void dropPos(){
        roller.setPower(0);
        wrist.setPosition(GripperPos.ROLLERDROP.value);
    }
    public void move() {
        roller.setPower(0);
        wrist.setPosition(GripperPos.WRIST_MOVE.value);
    }

    public void collect() {
        roller.setPower(RollerPower.COLLECT.value);
        wrist.setPosition(GripperPos.WRISTCOLLECT.value);
    }

    public void autoCollect() {
        roller.setPower(RollerPower.COLLECT.value);
        wrist.setPosition(GripperPos.AUTO_WRISTCOLLECT.value);
    }


    public void collectVertical() {
        roller.setPower(RollerPower.COLLECT.value);
        wrist.setPosition(GripperPos.WRISTRESET.value);
    }

    public void reset() {
        wrist.setPosition(GripperPos.WRISTRESET.value);
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