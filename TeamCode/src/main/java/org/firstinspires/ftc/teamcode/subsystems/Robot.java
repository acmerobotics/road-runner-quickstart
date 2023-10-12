package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Mechanism;

public class Robot extends Mechanism {

    PixelManipulator pixelManipulator;
    Drivebase drivebase;
    @Override
    public void init(HardwareMap hwMap) {
        pixelManipulator = new PixelManipulator();
        drivebase = new Drivebase();
    }

    @Override
    public void loop(Gamepad gamepad, Gamepad gamepad2) {
        pixelManipulator.loop(gamepad, gamepad2);
        pixelManipulator.loop(gamepad);
    }
}
