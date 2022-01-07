package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ScoringArm extends Mechanism{
    /****
     * WHAT TO DO HERE:
     * Figure out the DESIRED endpoints using ServoTest OpMode
     * Ensure that it's really what you want
     * Test it out, with low commitment
     * If it works, you're good to go
     ****/

    private static double armServoLStart = 1.0;
    private static double armServoLEnd = 0.6;
    private Arm portSide = new Arm("armServoL", armServoLStart, armServoLEnd) {
    };

    private static double armServoRStart = 0.0;
    private static double armServoREnd = 0.4;
    private Arm starBoard = new Arm("armServoR", armServoRStart, armServoREnd) {
    };

    private static double depositStart = 0.0;
    private static double depositEnd = 0.0;
    private Arm deposit = new Arm("deposit",depositStart,depositEnd);

    @Override
    public void init(HardwareMap hwMap) {
        portSide.init(hwMap);
        starBoard.init(hwMap);
        deposit.init(hwMap);
    }

    public void open(){
        deposit.startPos();
    }

    public void close(){
        deposit.endPos();
    }

    public void toggle(){
        if (deposit.atEnd()) deposit.startPos();
        else if (deposit.atStart()) deposit.startPos();
    }

    public void goTo(double desiredPosition){
        starBoard.setPosRatio(desiredPosition);
        portSide.setPosRatio(desiredPosition);
    }

    public void goToEnd(){
        starBoard.endPos();
        portSide.endPos();
    }

    public void goToStart(){
        starBoard.startPos();
        portSide.startPos();
    }

}
