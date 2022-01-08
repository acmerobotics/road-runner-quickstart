package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm extends ServoMechanism{
    //This class is meant to be a universal class for implementing arm like structures
    private Servo arm;
    private String hwMapName;
    private double armStart;
    private double armEnd;
    private boolean formerBool;

    //store the ROM and desired name of servo

    /**
     * Sets the arm's information reguarding ROM and name in hwmap
     * @param hwMapName String to name servo in hardware map
     * @param armStart start of range of servo
     * @param armEnd end of range of servo
     */
    public Arm(String hwMapName,double armStart, double armEnd){
        this.hwMapName = hwMapName;
        this.armStart = armStart;
        this.armEnd = armEnd;
    }

    /**
     * initializes stuff yk
     * @param hwMap robot's hardware map
     */
    public void init(HardwareMap hwMap) {
        arm = hwMap.servo.get(hwMapName);
    }

    /**
     * moves arm to armEnd
     */
    public void endPos(){
        arm.setPosition(armEnd);
    }

    /**
     * moves arm to armStart
     */
    public void startPos(){
        arm.setPosition(armStart);
    }

    /**
     * sets position of arm
     * @param position desired position in range of armStart-armEnd
     */
    public void setPos(double position){
        arm.setPosition(position);
    }

    /**
     * sets position of arm in terms of ratio
     * @param ratio desired position in range of 0.0-1.0 (converts from armStart-armEnd)
     */
    public void setPosRatio(double ratio){
        /*
        ratio, where ratio is a double from 0.0 to 1.0
         */
        double adjusted = (armEnd - armStart) * ratio + armStart;
        arm.setPosition(adjusted);

    }

    public void toggle(){
        if(atEnd()) startPos();
        else if (atStart()) endPos();
    }

    public double getPos() {return arm.getPosition();}

    /**
     *
     * @return endpoints, in format [armStart,armEnd]
     */
    public double[] getEndPoints(){
        return new double [] {armStart,armEnd};
    }

    public boolean atEnd(){
        return arm.getPosition() == armEnd;
    }

    public boolean atStart(){
        return arm.getPosition() == armStart;
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
