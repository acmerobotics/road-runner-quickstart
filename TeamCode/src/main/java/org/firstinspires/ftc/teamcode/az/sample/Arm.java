package org.firstinspires.ftc.teamcode.az.sample;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@Autonomous

@TeleOp
public class Arm extends LinearOpMode {

    public static final double POWER = 1.0;
    public static final int INCREMENT = 25;
    private DcMotor arm;
    LinearOpMode opMode;

    public Arm() {
        super();
    }

    public Arm(LinearOpMode newOpMode) {
        this.opMode = newOpMode;
        setup();
    }

    public void move() {
        AZUtil.setMotorTargetPosition(arm, ArmPos.MOVE.value, POWER);
    }

    public enum ArmPos {
        DROP(180),
        RESET(0),
        COLLECT(-1150),
        CARRY(0),
        SPECIMEN_HANG(160),

        MOVE(-700),
        BASKET_DROP(1000);



        private int value;

        ArmPos(int val) {
            this.value = val;
        }

        public double getValue() {
            return this.value;
        }
    }

    public void setup() {
        arm = opMode.hardwareMap.get(DcMotor.class, "arm");
        setupPos();
    }

    public void moveToPosition(int pos) {
       AZUtil.setMotorTargetPosition(arm,pos, POWER);
    }

    public void setupPos() {
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


    public void setArmPos(ArmPos pos){
        AZUtil.setMotorTargetPosition(arm, pos.value, POWER);
    }
    public void drop() {
        AZUtil.setMotorTargetPosition(arm, ArmPos.DROP.value, POWER);
    }

    public void collect() {
        AZUtil.setMotorTargetPosition(arm, ArmPos.COLLECT.value, POWER);
    }
    public void setPos(int pos){
        AZUtil.setMotorTargetPosition(arm, pos, POWER);
    }
    public void reset() {
        AZUtil.setMotorTargetPosition(arm, ArmPos.RESET.value,POWER);
    }

    public void moveUp(){
        int newPos = arm.getCurrentPosition() + INCREMENT;
        setPos(newPos);
    }
    public void specimenHang() {
        AZUtil.setMotorTargetPosition(arm, ArmPos.SPECIMEN_HANG.value, POWER);
    }



    public int getCurrentPos(){
        return arm.getCurrentPosition();
    }
    public void moveDown(){
        int newPos = arm.getCurrentPosition() - INCREMENT;
        setPos(newPos);
    }
    @Override
    public void runOpMode() {
        this.opMode = this;

        telemetry.addLine("Init");
        telemetry.update();
        setup();
        waitForStart();

        teleOpTest();
//        autoTest();

    }

    private void teleOpTest() {
        while(opModeIsActive()){

            if(gamepad1.dpad_up){
                setArmPos(ArmPos.BASKET_DROP);
            }
            if(gamepad1.dpad_down){
               setArmPos(ArmPos.RESET);
            }



            telemetry.addData("Pos", arm.getCurrentPosition());
            telemetry.update();
        }
    }

    private void autoTest() {
        setArmPos(ArmPos.RESET);
        sleep(3000);
        setArmPos(ArmPos.CARRY);
        sleep(3000);

        setArmPos(ArmPos.SPECIMEN_HANG);
        sleep(3000);

        setArmPos(ArmPos.BASKET_DROP);
        sleep(3000);

        reset();
        sleep(10000);
    }

}