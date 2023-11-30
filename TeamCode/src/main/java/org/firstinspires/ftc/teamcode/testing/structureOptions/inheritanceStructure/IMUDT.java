package org.firstinspires.ftc.teamcode.testing.structureOptions.inheritanceStructure;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class IMUDT extends MecDT {
    private IMU imu;
    private double forwards = 0;
    private double lastRead;
    int breaks;

    private RevHubOrientationOnRobot orientationOnRobot;

    void setIMU(IMU imu){
        this.imu = imu;
    }
    IMU getIMU(){
        return imu;
    }

    void initIMU(){
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }
    void initIMU(
            RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection,
            RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection
    ){
        orientationOnRobot = new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection);
        initIMU();
    }


    void setForwards(double direction){
        forwards = direction;
    }
    void setForwards(){
        imu.resetYaw();
        forwards = 0;
    }

    double getDirectionRaw(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
    double getDirection(){
        double direction = getDirectionRaw();
//        if(Double.isNaN(direction)){
//            direction = 0;
//            forwards -= lastRead;
//            initIMU();
//
//            breaks++;
//        }else{
//            lastRead = direction;
//        }
        return direction - forwards;
    }

    void absoluteDirectionalPow(double forward, double side, double rot){
        absoluteDirectionalPow(getDirection(), forward, side, rot);
    }
}
