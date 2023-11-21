package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Mechanism;

public class Robot extends Mechanism {

    public PixelManipulator pixelManipulator;
    public Drivebase drivebase;

    PAL pal;

    boolean isRedAlliance;

    public Robot(boolean isRedAlliance) {
        this.isRedAlliance = isRedAlliance;
    }
    @Override
    public void init(HardwareMap hwMap) {
        pixelManipulator = new PixelManipulator();
        drivebase = new Drivebase(this.isRedAlliance);
        pal = new PAL();

        pixelManipulator.init(hwMap);
        drivebase.init(hwMap);
        pal.init(hwMap);
    }

    @Override
    public void loop(Gamepad gamepad, Gamepad gamepad2) {
        pixelManipulator.loop(gamepad, gamepad2);
        drivebase.loop(gamepad);
        pal.loop(gamepad);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        pixelManipulator.telemetry(telemetry);
        drivebase.telemetry(telemetry);
    }
}
