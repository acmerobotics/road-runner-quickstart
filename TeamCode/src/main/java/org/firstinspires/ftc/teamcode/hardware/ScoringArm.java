package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ScoringArm extends ServoMechanism{
    /****
     * WHAT TO DO HERE:
     * Figure out the DESIRED endpoints using ServoTest OpMode
     * Ensure that it's really what you want
     * Test it out, with low commitment
     * If it works, you're good to go
     ****/

    //pivot arm setup
    private static double armServoLStart = 0.1;
    private static double armServoLEnd = 0.6;
    //0.8
    private static double armServoRStart = 0.98;
    private static double armServoREnd = 0.5;
    //0.3
    private GearedServos pivotArm = new GearedServos(
            "armServoL", armServoLStart, armServoLEnd,
            "armServoR", armServoRStart, armServoREnd
            );

    //deposit setup
    public static double depositStart = 0.1;
    public static double depositEnd = 0.5;
//    private Arm deposit = new Arm("deposit",depositStart,depositEnd);
    private Servo armServoL;
    private Servo armServoR;
    private Servo deposit;
    private boolean raised;
    @Override
    public void init(HardwareMap hwMap) {
        armServoL = hwMap.servo.get("armServoL");
        armServoR = hwMap.servo.get("armServoR");
        deposit = hwMap.servo.get("deposit");
        goToStart();
    }

//    public void open(){
//        deposit.startPos();
//    }
//
//    public void close(){
//        deposit.endPos();
//    }
//
//    public void toggle(){
//        if (deposit.atEnd()) deposit.startPos();
//        else if (deposit.atStart()) deposit.startPos();
//    }

    public void goTo(double desiredPosition){
    }

    public void goToEnd(){
        armServoL.setPosition(armServoLEnd);
        armServoR.setPosition(armServoREnd);
        raised = true;
    }

    public void goToStart(){
        armServoL.setPosition(armServoLStart);
        armServoR.setPosition(armServoRStart);
        raised = false;
    }

    public void run(boolean bool){
        if(bool){
            goToStart();
        }
        else{
            goToEnd();
        }
    }

    public void toggleDeposit(boolean bool){
        if(raised){
            if(bool) {
                deposit.setPosition(depositStart);
            } else{
                deposit.setPosition(depositEnd);
            }
        }
    }

    public void runDeposit(boolean bool){

    }

}
