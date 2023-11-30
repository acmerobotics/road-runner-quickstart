package org.firstinspires.ftc.teamcode.testing.structureOptions.inheritanceStructure;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class Hardware {
    private IMUDT drivetrain = new IMUDT();
    private WebcamName webcamName;

    void init(HardwareMap hardwareMap){
        drivetrain.setMotors(
                hardwareMap.get(DcMotor.class, "leftfront_drive"),
                hardwareMap.get(DcMotor.class, "rightfront_drive"),
                hardwareMap.get(DcMotor.class, "leftback_drive"),
                hardwareMap.get(DcMotor.class, "rightback_drive")
        );
        drivetrain.setDirections(
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.FORWARD
        );
        drivetrain.setIMU(
                hardwareMap.get(IMU.class, "imu")
        );
        drivetrain.initIMU(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
    }

    IMUDT getDrivetrain(){
        return drivetrain;
    }
}
