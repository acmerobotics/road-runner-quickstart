package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class GearedServos extends ServoMechanism{
    /****
     *TODO:
     * Figure out the DESIRED endpoints using ServoTest OpMode
     * Ensure that it's really what you want
     * Test it out, with low commitment
     *      e.g MAKE SURE THEY'RE NOT SCREWED IN OR GEARED FROM THE GET GO
     *      IF THEY ARE GEARED TOGETHER, MAKE SURE IT WONT SCREW THINGS UP
     * If it works, you're good to go
     * NOTE: Starboard is the right side of the ship, and portside is the right side of the ship,
     *       when considering the ship from the orientation in which it moves forwards
     * NOTE: The two servos do not necessarily need to be geared, but only to move in tandem with
     *       each other, such that they move in OPPOSITE DIRECTIONS (Ex. one rotates clockwise and the
     *       other rotates counter clockwise) and at the SAME SPEED (a given if the servos are the same,
     *       or if a workaround is used, like gear ratios)
     ****/


    private ServoManager portSide; //left side
    private ServoManager starBoard; //right side
    private boolean formerBool = false;

    /**
     *
     * @param armServoRName hardwaremap name of starboard servo
     * @param armServoRStart starting endpoint of starboard servo
     * @param armServoREnd ending endpoint of starboard servo
     * @param armServoLName hardwaremap name of portside servo
     * @param armServoLStart starting endpoint of portside servo
     * @param armServoLEnd ending endpoint of portside servo
     */
    public GearedServos(String armServoRName, double armServoRStart, double armServoREnd,
                        String armServoLName, double armServoLStart, double armServoLEnd){
        portSide = new ServoManager(armServoLName, armServoLStart, armServoLEnd) {
        };
        starBoard = new ServoManager(armServoRName, armServoRStart, armServoREnd) {
        };

    }

    /**
     *
     * @param hwMap robot's hardware map
     */
    @Override
    public void init(HardwareMap hwMap) {
        portSide.init(hwMap);
        starBoard.init(hwMap);
        //TODO: SET THE STARTING POINT OF THE GEARED-SERVOS AS DESIRED USING THE DOUBLE VARIABLE
        double startingPosition = 0.0;

        portSide.setPosRatio(startingPosition);
        starBoard.setPosRatio(startingPosition);
    }


    /**
     *
     * @param desiredPosition moves to desires position IN TERMS OF RATIO, where 0 is the start points of both servos, and 1 is the end point of both servos
     */
    public void goTo(double desiredPosition){
        starBoard.setPosRatio(desiredPosition);
        portSide.setPosRatio(desiredPosition);
    }

    /**
     * moves to the end position of both servos
     */
    public void goToEnd(){
        starBoard.endPos();
        portSide.endPos();
    }

    /**
     * moves to start position of both servos
     */
    public void goToStart(){
        starBoard.startPos();
        portSide.startPos();
    }

    /**
     * toggles back and forth between end and start positions
     */
    public void toggle(){
        starBoard.toggle();
        portSide.toggle();
    }

    /**
     * run command activiates toggle upon button press
     * @param bool
     */
    public void run(boolean bool) {
        if (bool) formerBool = true;
        if (formerBool) {
            if (!bool){
                toggle();
                formerBool = false;
            }
        }

    }

    public double getPos(){
        return portSide.getPos();
    }
}
