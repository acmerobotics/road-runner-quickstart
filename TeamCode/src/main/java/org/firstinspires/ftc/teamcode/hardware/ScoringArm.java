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

    //arm servo positions
    private static double armServoLStart = 0.1;
    private static double armServoLMid = 5/7;
    private static double armServoLEnd = 0.8;

    private static double armServoRStart = 0.98;
    private static double armServoRMid = 5/7;
    private static double armServoREnd = 0.3;

    private GearedServos pivotArm = new GearedServos(
            "armServoR", armServoRStart, armServoREnd,
            "armServoL", armServoLStart, armServoLEnd
            );

    //deposit servo positions
    public static double depositStart = 0.1;
    public static double depositEnd = 0.5;
    private Arm deposit = new Arm("deposit",depositStart,depositEnd);
    private boolean homed;
    @Override
    public void init(HardwareMap hwMap) {
        pivotArm.init(hwMap);
        deposit.init(hwMap);
        homed = true;
    }
    //GO TO POS RATIO
    public void goTo(double desiredPosition){
        pivotArm.goTo(desiredPosition);
        homed = false;
    }
    // MAX
    public void goToEnd(){
        pivotArm.goToEnd();
        homed = false;
    }
    //RESET
    public void goToStart(){
        pivotArm.goToStart();
        homed = true;
    }

    public void dump(){
        //DUMP ONLY IF HOMED = FALSE
    }
    public void run(boolean bool){
        if(bool){
            goToStart();
        }
        else{
            goToEnd();
        }
    }


}
