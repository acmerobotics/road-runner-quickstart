package org.firstinspires.ftc.teamcode.Simulation.TestCommandsSubsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;

public class PrintSubsystem1 extends Subsystem {
    public void initAuto(HardwareMap hwMap) {
        print("PrintSubsystem Init Complete");
    }

    public void periodic() {

    }

    public void shutdown() {
        print("PrintSubsystem Shutdown Complete");
    }

    public void print(String message) {
        System.out.println(message);
    }
}
