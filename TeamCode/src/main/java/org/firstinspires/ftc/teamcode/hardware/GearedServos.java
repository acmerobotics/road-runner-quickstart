package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class GearedServos extends ServoMechanism{
    /****
     * WHAT TO DO HERE:
     * Figure out the DESIRED endpoints using ServoTest OpMode
     * Ensure that it's really what you want
     * Test it out, with low commitment
     *      e.g MAKE SURE THEY'RE NOT SCREWED IN OR GEARED FROM THE GET GO
     *      IF THEY ARE GEARED TOGETHER, MAKE SURE IT WONT SCREW THINGS UP
     * If it works, you're good to go
     ****/


    private Arm portSide;
    private Arm starBoard;
    private boolean formerBool = false;

    public GearedServos(String armServoRName, double armServoRStart, double armServoREnd,
                        String armServoLName, double armServoLStart, double armServoLEnd){
        portSide = new Arm(armServoLName, armServoLStart, armServoLEnd) {
        };
        starBoard = new Arm(armServoRName, armServoRStart, armServoREnd) {
        };

    }
    @Override
    public void init(HardwareMap hwMap) {
        portSide.init(hwMap);
        starBoard.init(hwMap);
        portSide.atStart();
        starBoard.atStart();
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

    public void toggle(){
        starBoard.toggle();
        portSide.toggle();
    }

    public void run(boolean bool) {
        if (bool) formerBool = true;
        if (formerBool) {
            if (!bool){
                toggle();
                formerBool = false;
            }
        }

    }
}
