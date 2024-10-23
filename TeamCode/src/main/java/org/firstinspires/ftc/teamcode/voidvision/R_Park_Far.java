package org.firstinspires.ftc.teamcode.voidvision;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Far Park Red",group="Encoder")
public class R_Park_Far extends Auto_Util {
    public void runOpMode() throws InterruptedException {
        initAuto();

        waitForStart();
        encoderDrive(DRIVE_SPEED,4,4,10,0);
        encoderStrafe(STRAFE_SPEED,5,5,10,0);

    }
}