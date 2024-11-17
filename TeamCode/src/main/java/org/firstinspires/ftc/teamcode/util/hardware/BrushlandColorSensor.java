package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BrushlandColorSensor extends Component {
    DigitalChannel pin0;
    DigitalChannel pin1;
    public BrushlandColorSensor(int port, String name, HardwareMap map) {
        super(port, name);
        pin0 = map.digitalChannel.get(name + "p0");
        pin1 = map.digitalChannel.get(name + "p1");
    }

    public boolean getPin0() {
        return pin0.getState();
    }

    public boolean getPin1() {
        return pin1.getState();
    }

    public boolean getBoth() {
        return getPin0() && getPin1();
    }

    public boolean getEither() {
        return getPin0() || getPin1();
    }

    public boolean getNeither() {
        return !getPin0() && !getPin1();
    }

    public boolean onlyPin0() {
        return getPin0() && !getPin1();
    }

    public boolean onlyPin1() {
        return getPin1() && !getPin0();
    }
}
