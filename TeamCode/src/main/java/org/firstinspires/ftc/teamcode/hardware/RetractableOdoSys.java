package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.C;
@Config
public class RetractableOdoSys extends ServoMechanism{

    //TODO: Find the limits of the individual servos using ServoTest OpMode.
    // * Set the hwmap names first hand. It is suggested to NOT ASSEMBLE SERVOS until you are CERTAIN of the endpoints.
    // * The LIMIT_____START is the point of RETRACTION. the LIMIT____END is the point of RELEASE
    // * Ensure that at release there is MORE than the minimum required to enable the servo the ability to hang freely
    // * Once the limits are found, set them here.
    // * You can test in the OpMode
    // * To call the method on your own, utilize run(boolean). Ex. ```odoSys.run(gamepad1.a)```

    private boolean formerBool;
    private boolean released = true;

    //LEFT LIMITS HERE
    public static double LIMIT_LEFT_START = 0.42;
    public static double LIMIT_LEFT_END = 1.0;
    ServoManager left = new ServoManager("odoLeft",LIMIT_LEFT_START,LIMIT_LEFT_END);

    //RIGHT LIMITS HERE
    public static double LIMIT_RIGHT_START = 0.95;
    public static double LIMIT_RIGHT_END = 0.45;
    ServoManager right = new ServoManager("odoRight",LIMIT_RIGHT_START,LIMIT_RIGHT_END);

    //FRONT LIMITS HERE
    public static double LIMIT_FRONT_START = 0.25;
    public static double LIMIT_FRONT_END = 0.85;
    ServoManager front = new ServoManager("odoFront",LIMIT_FRONT_START,LIMIT_FRONT_END);

    ServoManager[] odoRetractors = {left,right,front};
    @Override
    public void init(HardwareMap hwMap) {
        left.init(hwMap);
        right.init(hwMap);
        front.init(hwMap);

///        left = hwMap.servo.get("leftServo");
//        right = hwMap.servo.get("rightServo");
//        front = hwMap.servo.get("frontServo");
    }

    public void init(HardwareMap hwMap, boolean auton){
        init(hwMap);
        if(auton){
            release();
        }
        else{
            retract();
        }
    }

    @Override
    @Deprecated
    public void run(boolean bool) {
        if(bool){
            formerBool = true;
        }

        if(formerBool & !bool){
            toggle();
        }
    }

    /**
     * retracts odo
     */
    private void retract(){
        for(int i = 0; i < odoRetractors.length;i++){
            odoRetractors[i].startPos();
        }
        released = false;
    }

    /**
     * releases odo
     */
    private void release(){
        for(int i = 0; i < odoRetractors.length;i++){
            odoRetractors[i].endPos();
        }
        released = true;
    }

    /**
     *
     * @return true if odo is released
     */
    public boolean isReleased(){
        return released;
    }

    /**
     * toggles between release and retract
     */
    public void toggle(){
        if(isReleased()){
            retract();
        }
        else{
            release();

        }
    }

    public void updateEndPoints(HardwareMap hwMap){
        left = new ServoManager("odoLeft",LIMIT_LEFT_START,LIMIT_LEFT_END);
        right = new ServoManager("odoRight",LIMIT_RIGHT_START,LIMIT_RIGHT_END);
        front = new ServoManager("odoFront",LIMIT_FRONT_START,LIMIT_FRONT_END);

        left.init(hwMap);
        right.init(hwMap);
        front.init(hwMap);

    }
}
