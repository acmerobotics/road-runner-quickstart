package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareDevice;

public class HardwareInterface{
    String deviceName;
    boolean isControlHub;
    int port;

    public HardwareInterface(String deviceName, boolean isControlHub, int port) {
        this.deviceName = deviceName;
        this.isControlHub = isControlHub;
        this.port = port;
    }

    public String getDeviceName() {
        return deviceName;
    }

    public boolean isControlHub() {
        return isControlHub;
    }

    public int getPort() {
        return port;
    }
}
