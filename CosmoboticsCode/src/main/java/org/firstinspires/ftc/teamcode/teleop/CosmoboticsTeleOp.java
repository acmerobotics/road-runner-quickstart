package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.DriveLogic;
import org.firstinspires.ftc.teamcode.teleop.MiscLogic;
import org.firstinspires.ftc.teamcode.teleop.TransportLogic;
@TeleOp
public class CosmoboticsTeleOp extends OpMode {
    DriveLogic driveLogic;
    TransportLogic transportLogic;
    MiscLogic miscLogic;

    @Override
    public void init() {
        driveLogic = new DriveLogic(hardwareMap);
        transportLogic = new TransportLogic(hardwareMap);
        miscLogic = new MiscLogic(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.update();
    }
}
