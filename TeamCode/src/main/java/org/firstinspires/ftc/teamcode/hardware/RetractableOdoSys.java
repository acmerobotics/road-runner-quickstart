package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
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
    public static double LIMIT_LEFT_START = 0.0;
    public static double LIMIT_LEFT_END = 0.0;
    ServoManager left = new ServoManager("LEFTHWMAPNAMEHERE",LIMIT_LEFT_START,LIMIT_LEFT_END);

    //RIGHT LIMITS HERE
    public static double LIMIT_RIGHT_START = 0;
    public static double LIMIT_RIGHT_END = 0;
    ServoManager right = new ServoManager("RIGHTHWMAPNAMEHERE",LIMIT_RIGHT_START,LIMIT_RIGHT_END);

    //FRONT LIMITS HERE
    public static double LIMIT_FRONT_START = 0;
    public static double LIMIT_FRONT_END = 0;
    ServoManager front = new ServoManager("FRONTHWMAPNAMEHERE",LIMIT_FRONT_START,LIMIT_FRONT_END);

    ServoManager[] odoRetractors = {left,right,front};
    @Override
    public void init(HardwareMap hwMap) {
        left.init(hwMap);
        right.init(hwMap);
        front.init(hwMap);


//        left = hwMap.servo.get("leftServo");
//        right = hwMap.servo.get("rightServo");
//        front = hwMap.servo.get("frontServo");
    }

    @Override
    public void run(boolean bool) {
        if(bool){
            formerBool = true;
        }

        if(formerBool & !bool){
            toggle();
        }
    }

    private void retract(){
        for(int i = 0; i < odoRetractors.length;i++){
            odoRetractors[i].startPos();
        }
    }

    private void release(){
        for(int i = 0; i < odoRetractors.length;i++){
            odoRetractors[i].endPos();
        }
    }

    public boolean isReleased(){
        return released;
    }

    public void toggle(){
        if(isReleased()){
            retract();
        }
        else{
            release();
        }
    }
}
