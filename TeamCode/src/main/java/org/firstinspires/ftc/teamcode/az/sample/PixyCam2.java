package org.firstinspires.ftc.teamcode.az.sample;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
//Declare a new I2cDeviceSync
//Get name from hardware config
@Autonomous
public class PixyCam2 extends LinearOpMode {
    I2cDeviceSynch pixyCam;
    @Override
    public void runOpMode() throws InterruptedException {

        pixyCam = hardwareMap.get(I2cDeviceSynch.class,"pixyCam");
        //pixyCam = hardwareMap.get("pixyCam");
        pixyCam.engage();
        waitForStart();
        while(opModeIsActive()) {
            byte[] sign1 = pixyCam.read(0x51, 5); // Read 5 bytes
            byte[] sign2 = pixyCam.read(0x52, 5); // Read 5 bytes
            // Assuming sign1 and sign2 are byte arrays
            int xValueSign1 = sign1[1] & 0xFF; // Extract the second byte, mask with 0xFF for unsigned value
            int xValueSign2 = sign2[1] & 0xFF; // Same for sign2
            telemetry.addData("X value of sign1", xValueSign1);
            telemetry.addData("X value of sign2", xValueSign2);
            telemetry.update();
        }
    }
}
