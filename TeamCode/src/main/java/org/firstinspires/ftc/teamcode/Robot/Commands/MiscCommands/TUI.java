package org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Utils.VirtualField;

public class TUI extends Command {
    Telemetry telemetry;
    VirtualField virtualField;

    public TUI(Telemetry telemetry) {
        this.telemetry = telemetry;
        virtualField = new VirtualField(telemetry);
    }

    @Override
    public void init() {
        System.out.println("TUI initialized");
    }

    @Override
    public void periodic() {
        virtualField.setPole(VirtualField.PoleState.RED);
        virtualField.moveCursor(1, 0);

        virtualField.draw();
    }

    @Override
    public boolean completed() {
        return false;
    }

    @Override
    public void shutdown() {
        System.out.println("TUI shutdown");
    }
}
