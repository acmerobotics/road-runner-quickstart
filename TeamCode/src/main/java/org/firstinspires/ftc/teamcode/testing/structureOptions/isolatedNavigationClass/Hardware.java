package org.firstinspires.ftc.teamcode.testing.structureOptions.isolatedNavigationClass;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Hardware {
    private MecDT DT = new MecDT();
    private WebcamName webcamName;
    private IMU imu;
    private RevHubOrientationOnRobot orientationOnRobot;

    void init(HardwareMap hardwareMap){
        DT.setMotors(
                hardwareMap.get(DcMotor.class, "leftfront_drive"),
                hardwareMap.get(DcMotor.class, "rightfront_drive"),
                hardwareMap.get(DcMotor.class, "leftback_drive"),
                hardwareMap.get(DcMotor.class, "rightback_drive")
        );
        DT.setDirections(
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD
        );


        imu = hardwareMap.get(IMU.class, "imu");
        initIMU(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
    }

    MecDT getDT(){
        return DT;
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
    double getDirection(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
}
