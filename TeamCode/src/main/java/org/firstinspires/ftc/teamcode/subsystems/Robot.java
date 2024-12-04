package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.gb.pinpoint.driver.Pose2D;

public class Robot extends Mechanism {

    public Drivebase drivebase;
    public Hubs hubs;
    public ScoringSystem scoringSystem;

    boolean isRed;
    Pose2D startingPosition;

    double oldTime = 0;

    public Robot(boolean isRed, Pose2D startingPosition) {
        this.isRed = isRed;
        this.startingPosition = startingPosition;
    }

    @Override
    public void init(HardwareMap hwMap) {
        drivebase = new Drivebase(startingPosition);
        hubs = new Hubs();
        scoringSystem = new ScoringSystem(isRed);

        drivebase.init(hwMap);
        hubs.init(hwMap);
        scoringSystem.init(hwMap);
    }

    @Override
    public void loop(AIMPad gamepad1, AIMPad gamepad2) {
        drivebase.loop(gamepad1);
        scoringSystem.loop(gamepad1, gamepad2);
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        drivebase.telemetry(telemetry);
        scoringSystem.telemetry(telemetry);
    }
}
